
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"



/* Private includes ----------------------------------------------------------*/
//#include <stdio.h>

#include "Buzzer.h"
#include "ssd1306.h"
#include "fonts.h"
#include "keyboard.h"
#include "accelerometer.h"
#include "helper.h"
#include "nmea_parser.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"




/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osTask<64> defaultTask("defaultTask");
osTask<64> buzzerTask("buzzerTask");
osTask<128> keyboardTask("keyboardTask");
osTask<196> dysplayTask("dysplayTask");
osTask<64> accelTask("accelTask");
osTask<256> gpsNMEA_ParserT("gpsNMEA_ParserT");
osTask<256> sdCardTask("sdCardTask",osPriorityAboveNormal3);


osQueue<buzzerStruct,2>  buzzerQueueHandle("buzzerQueue");

osQueue<uint16_t,16> dysplayQueueHandle("dysplayQueue");
osQueue<buttonStruct,1> keyboardQueueHandle("keyboardQueue");
osQueue<uint8_t,64>  GPS_UARTQueueHandle("GPS_UARTQueue");



/* Definitions for I2C_BinarySem */
osSemaphoreId_t I2C_BinarySemHandle;
const osSemaphoreAttr_t I2C_BinarySem_attributes = {
		.name = "I2C_BinarySem"
};
/* Definitions for accelStructBinarySem */


osSemaphoreId_t accelStructBinarySemHandle;
const osSemaphoreAttr_t accelStructBinarySem_attributes = {
		.name = "accelStructBinarySem"
};

osSemaphoreId_t debugUARTBinarySemHandle;
const osSemaphoreAttr_t  debugUARTBinarySem_attributes = {
		.name = " debugUARTBinarySem"
};
/* USER CODE BEGIN PV */

TM_MPU9250_t accelStruct;
uint8_t receiveBuffer[32];
//NMEA_UART::Parser *gpsParser = new NMEA_UART::Parser();
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
void StartAccelTask(void *argument);
void StartgpsNMEA_ParserTask(void *argument);
void StartSDcardTask(void *argument);
void StartDebugUARTTask(void *argument);
void debugMsg(const char* msg, bool withTime, bool newLine);




int main(void)
{

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();

	BuzzerSetVolume(0);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(50);
	ssd1306_Init();
	HAL_Delay(50);
	ssd1306_Fill(Black);
	HAL_Delay(50);
	ssd1306_UpdateScreen();
	HAL_Delay(50);
	TM_MPU9250_Result_t res =  MPU9250_Init(&hi2c1,&accelStruct,TM_MPU9250_Device_0);

	//USART1->CR1 |= USART_CR1_TCIE; /*//прерывание по окончанию передачи*/
	USART1->CR1 |= USART_CR1_RXNEIE; /*//прерывание по приему данных*/
	HAL_UART_Receive_IT (&huart1, receiveBuffer, (uint8_t) 1);




	/* Init scheduler */
	osKernelInitialize();

	I2C_BinarySemHandle = osSemaphoreNew(1, 1, &I2C_BinarySem_attributes);
	accelStructBinarySemHandle = osSemaphoreNew(1,1, &accelStructBinarySem_attributes);
	debugUARTBinarySemHandle = osSemaphoreNew(1,1, &debugUARTBinarySem_attributes);

	buzzerQueueHandle.createQueue();
	dysplayQueueHandle.createQueue();
	keyboardQueueHandle.createQueue();
	GPS_UARTQueueHandle.createQueue();


	/* creation of defaultTask */

	defaultTask.start(StartDefaultTask);

	buzzerTask.start(StartBuzzerTask);				//Handle = osThreadNew(StartBuzzerTask, NULL, &buzzerTask_attributes);
	keyboardTask.start(StartKeyboardTask);			//Handle = osThreadNew(StartKeyboardTask, NULL, &keyboardTask_attributes);
	dysplayTask.start(StartDysplayTask);			//Handle = osThreadNew(StartDysplayTask, NULL, &dysplayTask_attributes);
	accelTask.start(StartAccelTask);				//Handle = osThreadNew(StartAccelTask, NULL, &accelTask_attributes);
	gpsNMEA_ParserT.start(StartgpsNMEA_ParserTask);	//Handle = osThreadNew(StartgpsNMEA_ParserTask, NULL, &gpsNMEA_ParserT_attributes);
	sdCardTask.start(StartSDcardTask);

	/* Start scheduler */
	osKernelStart();

	while (1)
	{

	}
}

void debugMsg(const char* msg, bool withTime, bool newLine)
{
	int tick = osKernelGetTickCount();
	char tickLine[24];
	char newline[] = "\r\n";
	char dot = ':';
	sprintf(tickLine, "%d", tick);


	osSemaphoreAcquire(debugUARTBinarySemHandle,osWaitForever);

	if(withTime){
		HAL_UART_Transmit(&huart3,(uint8_t*)tickLine , strlen(tickLine), 10U);
		HAL_UART_Transmit(&huart3,(uint8_t*)&dot , 1, 10U);
	}

	HAL_UART_Transmit(&huart3,(uint8_t*)msg , strlen(msg), 10U);

	if (newLine) HAL_UART_Transmit(&huart3, (uint8_t *) newline, 2, 10);
	osSemaphoreRelease(debugUARTBinarySemHandle);

}

void StartSDcardTask(void *argument)
{



	for(;;)
	{
		osDelay(20);
	}
}




void StartDefaultTask(void *argument)
{
	/* init code for USB_DEVICE */

	/* USER CODE BEGIN 5 */
	osStatus_t keyStatus;
	buttonStruct buttonEvent;


	/* Infinite loop */
	for(;;)
	{
		keyStatus =keyboardQueueHandle.receive(&buttonEvent);// osMessageQueueGet(keyboardQueueHandle, &buttonEvent, NULL, 1U);   // wait for message
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

		}

		osDelay(20);


	}

}


/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
	/* USER CODE BEGIN StartBuzzerTask */
	/* Infinite loop */
	buzzerStruct buzzerParameters;
	osStatus_t status;
	for(;;)
	{
		status = buzzerQueueHandle.receive(&buzzerParameters); // osMessageQueueGet(buzzerQueueHandle, &buzzerParameters, NULL, osWaitForever);   // wait for message
		if (status == osOK) {
			BuzzerSetFreq(buzzerParameters.freq);
			BuzzerSetVolume(buzzerParameters.volume);
			osDelay(buzzerParameters.duration);
			BuzzerSetVolume(0);
		}
	}
	/* USER CODE END StartBuzzerTask */
}

void StartKeyboardTask(void *argument)
{
	/* USER CODE BEGIN StartKeyboardTask */
	buzzerStruct buzzerPress;
	buzzerPress.duration=5;
	buzzerPress.freq=3000;
	buzzerPress.volume=BUZZER_VOLUME_MAX;
	buzzerStruct buzzerRELESE;
	buzzerRELESE.duration=5;
	buzzerRELESE.freq=3500;
	buzzerRELESE.volume=BUZZER_VOLUME_MAX;
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
			buzzerQueueHandle.send(buzzerPress);
			buttonEvent.buttonNumber=1;
			buttonEvent.state = BUTTON_SHORT_PRESSED;


			keyboardQueueHandle.send(buttonEvent);//osMessageQueuePut(keyboardQueueHandle, &buttonEvent, 0U, 0U);
		}

		if(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET && (!button_one_flag))
		{
			buzzerQueueHandle.send(buzzerRELESE);
			buttonEvent.buttonNumber=1;
			buttonEvent.state = BUTTON_RELEASED;
			keyboardQueueHandle.send(buttonEvent);
		}

		if(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET)
		{
			button_one_flag=1;
		}

		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET && button_two_flag)
		{
			button_two_flag=0;
			buzzerQueueHandle.send(buzzerPress);
			buttonEvent.buttonNumber=2;
			buttonEvent.state = BUTTON_SHORT_PRESSED;
			keyboardQueueHandle.send(buttonEvent);
		}

		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET && (!button_two_flag))
		{
			buzzerQueueHandle.send(buzzerRELESE);
			buttonEvent.buttonNumber=2;
			buttonEvent.state = BUTTON_RELEASED;
			keyboardQueueHandle.send(buttonEvent);
		}

		if(HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET)
		{
			button_two_flag=1;
		}

		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_RESET && button_three_flag)
		{
			button_three_flag=0;
			buzzerQueueHandle.send(buzzerPress);

			buttonEvent.buttonNumber=3;
			buttonEvent.state = BUTTON_SHORT_PRESSED;
			keyboardQueueHandle.send(buttonEvent);
		}

		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET && (!button_three_flag))
		{

			buzzerQueueHandle.send(buzzerRELESE);

			buttonEvent.buttonNumber=3;
			buttonEvent.state = BUTTON_RELEASED;
			keyboardQueueHandle.send(buttonEvent);
		}


		if(HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET)
		{
			button_three_flag=1;
		}

		//HAL_GPIO_ReadPin(BTN1_PORT, BTN1_PIN) == GPIO_PIN_RESET;
		//HAL_GPIO_ReadPin(BTN2_PORT, BTN2_PIN) == GPIO_PIN_RESET;
		//HAL_GPIO_ReadPin(BTN3_PORT, BTN3_PIN) == GPIO_PIN_RESET;



		osDelay(5);
	}
	/* USER CODE END StartKeyboardTask */
}

/* USER CODE END Header_StartDysplayTask */
void StartDysplayTask(void *argument)
{
	/* USER CODE BEGIN StartDysplayTask */
	/* Infinite loop */



	for(;;)
	{
		osSemaphoreAcquire(accelStructBinarySemHandle,osWaitForever);
		char str[32];
		//std::string ssa;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(2,2);
		sprintf(str, "%d", accelStruct.Ax_Raw);
		ssd1306_WriteString("Ax:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//debugMsg(" Ax:",0,0);
		//debugMsg(str,0,0);

		ssd1306_SetCursor(2,12);
		sprintf(str, "%d", accelStruct.Ay_Raw);
		ssd1306_WriteString("Ay:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//debugMsg(" Ay:",0,0);
		//debugMsg(str,0,0);

		ssd1306_SetCursor(2,22);
		sprintf(str, "%d", accelStruct.Az_Raw);
		ssd1306_WriteString("Az:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" Az:",0,0);
		//	debugMsg(str,0,0);

		ssd1306_SetCursor(60,2);
		sprintf(str, "%d", accelStruct.Gx_Raw);
		ssd1306_WriteString("Gx:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//debugMsg( " Gx:",0,0);
		//	debugMsg(str,0,0);

		ssd1306_SetCursor(60,12);
		sprintf(str, "%d", accelStruct.Gy_Raw);
		ssd1306_WriteString("Gy:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" Gy:",0,0);
		//			debugMsg(str,0,0);

		ssd1306_SetCursor(60,22);
		sprintf(str, "%d", accelStruct.Gz_Raw);
		ssd1306_WriteString("Gz:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" Gz:",0,0);
		//			debugMsg(str,0,0);

		ssd1306_SetCursor(2,43);
		//ssd1306_WriteString(gpsParser->HDOP,Font_7x10,White);

		ssd1306_SetCursor(2,33);
		sprintf(str, "%d", accelStruct.Mx_Raw*50);
		ssd1306_WriteString("Mx:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" Mx:",0,0);
		//				debugMsg(str,0,0);
		ssd1306_SetCursor(2,43);
		sprintf(str, "%d", accelStruct.My_Raw*50);
		ssd1306_WriteString("My:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" My:",0,0);
		//	debugMsg(str,0,0);

		ssd1306_SetCursor(2,53);
		sprintf(str, "%d", accelStruct.Mz_Raw*50);
		ssd1306_WriteString("Mz:",Font_7x10,White);
		ssd1306_WriteString(str,Font_7x10,White);
		//	debugMsg(" Mz:",0,0);
		//				debugMsg(str,0,0);
		//				debugMsg(" ",0,1);
		/*	if(gpsParser->Status[0]=='V'){
						ssd1306_WriteChar(gpsParser->Status[0],Font_7x10,White);
					}else{

					ssa = gpsParser->UTCtime;
					ssd1306_WriteString(ssa.c_str(),Font_7x10,White);
					}*/
		osSemaphoreRelease(accelStructBinarySemHandle);



		ssd1306_UpdateScreen();
		osDelay(10);
	}
	/* USER CODE END StartDysplayTask */
}

/* USER CODE BEGIN Header_StartAccelTask */
/**
 * @brief Function implementing the accelTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAccelTask */
void StartAccelTask(void *argument)
{
	/* USER CODE BEGIN StartAccelTask */
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(accelStructBinarySemHandle, osWaitForever);
		TM_MPU9250_ReadAcce(&accelStruct);
		osDelay(1);
		TM_MPU9250_ReadGyro(&accelStruct);
		osDelay(1);
		TM_MPU9250_ReadMag(&accelStruct);
		osDelay(1);
		osSemaphoreRelease(accelStructBinarySemHandle);
		osDelay(2);
	}
	/* USER CODE END StartAccelTask */
}

/* USER CODE BEGIN Header_StartgpsNMEA_ParserTask */
/**
 * @brief Function implementing the gpsNMEA_ParserT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartgpsNMEA_ParserTask */
void StartgpsNMEA_ParserTask(void *argument)
{
	/* USER CODE BEGIN StartgpsNMEA_ParserTask */
	/* Infinite loop */
	osStatus_t status;

	NMEA_UART::GPS_MESSEGE_TYPE messageType;
	NMEA_UART::Parser gpsParser;
	for(;;)
	{
		using namespace NMEA_UART;
		uint8_t sym=0;
		status =GPS_UARTQueueHandle.receive(&sym);  //osMessageQueueGet(GPS_UARTQueueHandle, &sym, NULL, osWaitForever);   // wait for message
		if (status == osOK) {

			messageType  = gpsParser.charParser(sym);
			switch (messageType) {
			case  GPS_MESSEGE_TYPE::GPS_NULL:
				break;
			case GPS_MESSEGE_TYPE::GPS_PRMC:
				break;
			case GPS_MESSEGE_TYPE::GPS_NRMC:

				break;
			case GPS_MESSEGE_TYPE::GPS_PGGA:
				break;
			case GPS_MESSEGE_TYPE::GPS_NGGA:
				break;
			case GPS_MESSEGE_TYPE::GPS_PGSV:
				break;
			case GPS_MESSEGE_TYPE::GPS_LGSV:
				break;
			case GPS_MESSEGE_TYPE::GPS_PVTG:
				break;
			case GPS_MESSEGE_TYPE::GPS_NVTG:
				break;
			case GPS_MESSEGE_TYPE::GPS_GPGSA:
				break;
			case GPS_MESSEGE_TYPE::GPS_GNGSA:
				break;
			default:
				break;
			}

		}

	}
	/* USER CODE END StartgpsNMEA_ParserTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	  HAL_UART_Transmit(&huart3, (uint8_t *) "ERROR ERROR!!!", 14, 10);
	/* USER CODE END Error_Handler_Debug */
}
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
	hi2c1.Init.ClockSpeed = 400000;
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
	HAL_UART_Receive_IT (&huart1, receiveBuffer, (uint8_t) 1);
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
