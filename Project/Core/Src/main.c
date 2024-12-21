/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BlueLedTask */
osThreadId_t BlueLedTaskHandle;
const osThreadAttr_t BlueLedTask_attributes = {
  .name = "BlueLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GreenLedTask */
osThreadId_t GreenLedTaskHandle;
const osThreadAttr_t GreenLedTask_attributes = {
  .name = "GreenLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ConversionTask */
osThreadId_t ConversionTaskHandle;
const osThreadAttr_t ConversionTask_attributes = {
  .name = "ConversionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedControlTask */
osThreadId_t LedControlTaskHandle;
const osThreadAttr_t LedControlTask_attributes = {
  .name = "LedControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PA2Task */
osThreadId_t PA2TaskHandle;
const osThreadAttr_t PA2Task_attributes = {
  .name = "PA2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDQueue */
osMessageQueueId_t LEDQueueHandle;
const osMessageQueueAttr_t LEDQueue_attributes = {
  .name = "LEDQueue"
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue"
};
/* Definitions for LCDMutex */
osMutexId_t LCDMutexHandle;
const osMutexAttr_t LCDMutex_attributes = {
  .name = "LCDMutex"
};
/* Definitions for PushButtonGreenLedSem */
osSemaphoreId_t PushButtonGreenLedSemHandle;
const osSemaphoreAttr_t PushButtonGreenLedSem_attributes = {
  .name = "PushButtonGreenLedSem"
};
/* Definitions for GreenLedConversionSem */
osSemaphoreId_t GreenLedConversionSemHandle;
const osSemaphoreAttr_t GreenLedConversionSem_attributes = {
  .name = "GreenLedConversionSem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartBlueLedTask(void *argument);
void StartGreenLedTask(void *argument);
void StartConversionTask(void *argument);
void StartLedControlTask(void *argument);
void StartLCDTask(void *argument);
void StartPA2Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
      osSemaphoreRelease(PushButtonGreenLedSemHandle);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of LCDMutex */
  LCDMutexHandle = osMutexNew(&LCDMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PushButtonGreenLedSem */
  PushButtonGreenLedSemHandle = osSemaphoreNew(1, 0, &PushButtonGreenLedSem_attributes);

  /* creation of GreenLedConversionSem */
  GreenLedConversionSemHandle = osSemaphoreNew(1, 0, &GreenLedConversionSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LEDQueue */
  LEDQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &LEDQueue_attributes);

  /* creation of LCDQueue */
  LCDQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &LCDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of BlueLedTask */
  BlueLedTaskHandle = osThreadNew(StartBlueLedTask, NULL, &BlueLedTask_attributes);

  /* creation of GreenLedTask */
  GreenLedTaskHandle = osThreadNew(StartGreenLedTask, NULL, &GreenLedTask_attributes);

  /* creation of ConversionTask */
  ConversionTaskHandle = osThreadNew(StartConversionTask, NULL, &ConversionTask_attributes);

  /* creation of LedControlTask */
  LedControlTaskHandle = osThreadNew(StartLedControlTask, NULL, &LedControlTask_attributes);

  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* creation of PA2Task */
  PA2TaskHandle = osThreadNew(StartPA2Task, NULL, &PA2Task_attributes);

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
  while (1)
  {
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GreenLed_Pin|OrangeLed_Pin|RedLed_Pin|BlueLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenLed_Pin OrangeLed_Pin RedLed_Pin BlueLed_Pin */
  GPIO_InitStruct.Pin = GreenLed_Pin|OrangeLed_Pin|RedLed_Pin|BlueLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlueLedTask */
/**
* @brief Function implementing the BlueLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlueLedTask */
void StartBlueLedTask(void *argument)
{
  /* USER CODE BEGIN StartBlueLedTask */
  /* Infinite loop */
  for(;;)
  { HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    osDelay(500);
  }
  /* USER CODE END StartBlueLedTask */
}

/* USER CODE BEGIN Header_StartGreenLedTask */
/**
* @brief Function implementing the GreenLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenLedTask */
void StartGreenLedTask(void *argument)
{
  /* USER CODE BEGIN StartGreenLedTask */
	uint8_t counter=0x00;
  /* Infinite loop */
  for(;;)
  { if(counter==0)
  {
	osSemaphoreAcquire(PushButtonGreenLedSemHandle, osWaitForever);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    lcd_init();
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("FREERTOS Project");
    lcd_put_cur(1, 0);
    counter++;
  }
  else
  {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  }
  osSemaphoreRelease(GreenLedConversionSemHandle);
  osDelay(500);
  }
  /* USER CODE END StartGreenLedTask */
}

/* USER CODE BEGIN Header_StartConversionTask */
/**
* @brief Function implementing the ConversionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConversionTask */
void StartConversionTask(void *argument)
{
  /* USER CODE BEGIN StartConversionTask */
	uint16_t adc_val =0x00;
  /* Infinite loop */
  for(;;)
  { osSemaphoreAcquire(GreenLedConversionSemHandle,osWaitForever);
    if (HAL_ADC_Start(&hadc1)!=HAL_OK)
    {
    	osThreadExit();
    }
    if(HAL_ADC_PollForConversion(&hadc1, 1)!=HAL_OK)
    {
    	osThreadExit();
    }
    adc_val=HAL_ADC_GetValue(&hadc1);
    osMessageQueuePut(LEDQueueHandle, &adc_val, 0, osWaitForever);
    osMessageQueuePut(LCDQueueHandle, &adc_val, 0, 200);
    if (HAL_ADC_Stop(&hadc1)!=HAL_OK)
    	{
    	  osThreadExit();
    	}
        osDelay(1);
  }
  /* USER CODE END StartConversionTask */
}

/* USER CODE BEGIN Header_StartLedControlTask */
/**
* @brief Function implementing the LedControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedControlTask */
void StartLedControlTask(void *argument)
{
  /* USER CODE BEGIN StartLedControlTask */
	uint16_t get_adc2leds =0x00;
  /* Infinite loop */
  for(;;)
  {
   osMessageQueueGet(LEDQueueHandle, &get_adc2leds, 0, osWaitForever);
   if (get_adc2leds >4000)
   	  {
   	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
   	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
   	  }
   	  else
   	  {
   		if (get_adc2leds >2500)
   		{
   		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
   		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
   		}
   		else
   		{
   		  if (get_adc2leds >1500)
   		  {
   			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
   			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
   		  }
   		  else
   		  {
   			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
   			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
   		  }
   	    }
   	  }

  }
  osMutexRelease(LCDMutexHandle);
     osDelay(100);
  /* USER CODE END StartLedControlTask */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the LCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
	uint16_t get_adc2lcd = 0x00;
	uint8_t buffer[20];
  /* Infinite loop */
  for(;;)
  {
    if(osMutexAcquire(LCDMutexHandle, osWaitForever)==osOK)
    {
    	osMessageQueueGet(LCDQueueHandle, &get_adc2lcd, 0, osWaitForever);
    	lcd_put_cur(1, 0);
    	memset(buffer,' ', sizeof(buffer));
    	lcd_send_string(buffer);
    	sprintf(buffer,"La valeur: %d",get_adc2lcd);
    	lcd_put_cur(1, 0);
    	lcd_send_string(buffer);
    	osMutexRelease(LCDMutexHandle);
    }
    osDelay(50);
  }
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartPA2Task */
/**
* @brief Function implementing the PA2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPA2Task */
void StartPA2Task(void *argument)
{
  /* USER CODE BEGIN StartPA2Task */
	uint8_t buffer_urg[16];
	uint8_t count_lcd;
  /* Infinite loop */
  for(;;)
  {
	  if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)==GPIO_PIN_SET)
	  	  {
	  		 if (osMutexAcquire(LCDMutexHandle, osWaitForever) == osOK)
	  	    {
	  	      	lcd_put_cur(1,0);
	  	        lcd_send_string(" ...Urgence...  ");
	  	    }
	  		while( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)==GPIO_PIN_SET)
	  		 {
	  			  osDelay(100);
	  	     }
	  		 osMutexRelease(LCDMutexHandle);
	  	  }

	      osDelay(100);
  }
  /* USER CODE END StartPA2Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
