/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Buzzer.h"
#include "ssd1306.h"
#include "fonts.h"
#include "keyboard.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
		.name = "buzzerTask",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 128 * 4
};
/* Definitions for keyboardTask */
osThreadId_t keyboardTaskHandle;
const osThreadAttr_t keyboardTask_attributes = {
		.name = "keyboardTask",
		.priority = (osPriority_t) osPriorityBelowNormal,
		.stack_size = 128 * 4
};
/* Definitions for dysplayTask */
osThreadId_t dysplayTaskHandle;
const osThreadAttr_t dysplayTask_attributes = {
		.name = "dysplayTask",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 128 * 4
};
/* Definitions for buzzerQueue */
osMessageQueueId_t buzzerQueueHandle;
const osMessageQueueAttr_t buzzerQueue_attributes = {
		.name = "buzzerQueue"
};
/* Definitions for dysplayQueue */
osMessageQueueId_t dysplayQueueHandle;
const osMessageQueueAttr_t dysplayQueue_attributes = {
		.name = "dysplayQueue"
};
/* Definitions for keyboardQueue */
osMessageQueueId_t keyboardQueueHandle;
const osMessageQueueAttr_t keyboardQueue_attributes = {
		.name = "keyboardQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartBuzzerTask(void *argument);
void StartKeyboardTask(void *argument);
void StartDysplayTask(void *argument);

/* USER CODE BEGIN PFP */

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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_FATFS_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of buzzerQueue */
	buzzerQueueHandle = osMessageQueueNew (2, sizeof(buzzerStruct), &buzzerQueue_attributes);

	/* creation of dysplayQueue */
	dysplayQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &dysplayQueue_attributes);

	/* creation of keyboardQueue */
	keyboardQueueHandle = osMessageQueueNew (1, sizeof(buttonStruct), &keyboardQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of buzzerTask */
	buzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &buzzerTask_attributes);

	/* creation of keyboardTask */
	keyboardTaskHandle = osThreadNew(StartKeyboardTask, NULL, &keyboardTask_attributes);

	/* creation of dysplayTask */
	dysplayTaskHandle = osThreadNew(StartDysplayTask, NULL, &dysplayTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 479;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 254;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 124;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : SD_SPI_CS_Pin */
	GPIO_InitStruct.Pin = SD_SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SD_SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPS_EN_Pin */
	GPIO_InitStruct.Pin = GPS_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPS_PPS_Pin */
	GPIO_InitStruct.Pin = GPS_PPS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPS_PPS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MPU_INT_Pin */
	GPIO_InitStruct.Pin = MPU_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_3_Pin BTN_2_Pin BTN_1_Pin */
	GPIO_InitStruct.Pin = BTN_3_Pin|BTN_2_Pin|BTN_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_LE_Pin */
	GPIO_InitStruct.Pin = LED_LE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LED_LE_GPIO_Port, &GPIO_InitStruct);

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
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	osStatus_t keyStatus;
	buttonStruct buttonEvent;

	BuzzerSetVolume(0);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

	ssd1306_Init();
	HAL_Delay(1000);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_SetCursor(2,23);
	ssd1306_WriteString("Search GPS",Font_11x18,White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	/* Infinite loop */
	for(;;)
	{
		keyStatus = osMessageQueueGet(keyboardQueueHandle, &buttonEvent, NULL, 1U);   // wait for message
		if (keyStatus == osOK) {
			ssd1306_SetCursor(2,23);
			switch (buttonEvent.buttonNumber) {
			case 1:
				ssd1306_WriteString("1",Font_11x18,White);
				break;
			case 2:
				ssd1306_WriteString("2",Font_11x18,White);
				break;
			case 3:
				ssd1306_WriteString("3",Font_11x18,White);
				break;
			default:
				break;
			}

			ssd1306_WriteString(" ",Font_11x18,White);

			switch (buttonEvent.state) {
			case BUTTON_SHORT_PRESSED:
				ssd1306_WriteString("PRESS",Font_11x18,White);
				break;
			case BUTTON_RELEASED:
				ssd1306_WriteString("RELEAS",Font_11x18,White);
				break;
			default:
				break;
			}
			ssd1306_UpdateScreen();
		}

		osDelay(1);


	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
 * @brief Function implementing the buzzerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
	/* USER CODE BEGIN StartBuzzerTask */
	/* Infinite loop */
	buzzerStruct buzzerParameters;
	osStatus_t status;
	for(;;)
	{
		status = osMessageQueueGet(buzzerQueueHandle, &buzzerParameters, NULL, 0U);   // wait for message
		if (status == osOK) {
			BuzzerSetFreq(buzzerParameters.freq);
			BuzzerSetVolume(buzzerParameters.volume);
			osDelay(buzzerParameters.duration);
			BuzzerSetVolume(0);
		}



	}
	/* USER CODE END StartBuzzerTask */
}

/* USER CODE BEGIN Header_StartKeyboardTask */
/**
 * @brief Function implementing the keyboardTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartKeyboardTask */
void StartKeyboardTask(void *argument)
{
	/* USER CODE BEGIN StartKeyboardTask */
	buzzerStruct buzzerParameters;
	buttonStruct buttonEvent;
	uint8_t button_one_flag=1;
	uint8_t button_two_flag=1;
	uint8_t button_three_flag=1;
	/* Infinite loop */
	for(;;)
	{
		//
		if(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET && button_one_flag)
		{
			button_one_flag=0;
			buzzerParameters.duration=15;
			buzzerParameters.freq=3000;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);

			buttonEvent.buttonNumber=1;
			buttonEvent.state = BUTTON_SHORT_PRESSED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}
		if(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET && (!button_one_flag))
		{
			buzzerParameters.duration=15;
			buzzerParameters.freq=3500;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);
			buttonEvent.buttonNumber=1;
			buttonEvent.state = BUTTON_RELEASED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}

		if(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET)
		{
			button_one_flag=1;
		}



		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET && button_two_flag)
		{
			button_two_flag=0;
			buzzerParameters.duration=15;
			buzzerParameters.freq=3000;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);

			buttonEvent.buttonNumber=2;
			buttonEvent.state = BUTTON_SHORT_PRESSED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}

		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET && (!button_two_flag))
		{

			buzzerParameters.duration=15;
			buzzerParameters.freq=3500;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);
			buttonEvent.buttonNumber=2;
			buttonEvent.state = BUTTON_RELEASED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}

		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET)
		{
			button_two_flag=1;
		}


		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_RESET && button_three_flag)
		{
			button_three_flag=0;
			buzzerParameters.duration=15;
			buzzerParameters.freq=3000;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);

			buttonEvent.buttonNumber=3;
			buttonEvent.state = BUTTON_SHORT_PRESSED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}

		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET && (!button_three_flag))
		{

			buzzerParameters.duration=15;
			buzzerParameters.freq=3500;
			buzzerParameters.volume=BUZZER_VOLUME_MAX;
			osMessageQueuePut(buzzerQueueHandle, &buzzerParameters, 0U, 0U);

			buttonEvent.buttonNumber=3;
			buttonEvent.state = BUTTON_RELEASED;
			osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}


		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET)
		{
			button_three_flag=1;
		}

		//HAL_GPIO_ReadPin(BTN1_PORT, BTN1_PIN) == GPIO_PIN_RESET;
		//HAL_GPIO_ReadPin(BTN2_PORT, BTN2_PIN) == GPIO_PIN_RESET;
		//HAL_GPIO_ReadPin(BTN3_PORT, BTN3_PIN) == GPIO_PIN_RESET;


		osDelay(20);
	}
	/* USER CODE END StartKeyboardTask */
}

/* USER CODE BEGIN Header_StartDysplayTask */
/**
 * @brief Function implementing the dysplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDysplayTask */
void StartDysplayTask(void *argument)
{
	/* USER CODE BEGIN StartDysplayTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartDysplayTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
