
/* Includes ------------------------------------------------------------------*/
#include "defines.h"
#include "Buzzer.h"
#include "ssd1306.h"
#include "fonts.h"
#include "keyboard.h"
#include "accelerometer.h"
#include "nmea_parser.h"
#include "SDcard.h"

typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
		name : "buzzerTask",

		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 64 * 4,
		priority : (osPriority_t) osPriorityNormal,
		//	tz_module:NULL,  ///< TrustZone module identifier
		//	reserved:NULL  ///< reserved (must be 0)
};


osThreadId_t keyboardTaskHandle;
const osThreadAttr_t keyboardTask_attributes = {
		name : "keyboardTask",
		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 84 * 4,
		priority : (osPriority_t) osPriorityNormal,
};
osThreadId_t dysplayTaskHandle;
const osThreadAttr_t dysplayTask_attributes = {
		name : "dysplayTask",
		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 140 * 4,
		priority : (osPriority_t) osPriorityNormal,
};
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
		name : "accelTask",
		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 256 * 4,
		priority : (osPriority_t) osPriorityHigh7,
};

osThreadId_t gpsNMEA_ParserTHandle;
const osThreadAttr_t gpsNMEA_ParserT_attributes = {
		name : "gpsNMEA_ParserT",
		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 386 * 4,
		priority : (osPriority_t) osPriorityNormal1,
};
osThreadId_t sdCardTaskHandle;
const osThreadAttr_t sdCardTask_attributes = {
		name : "sdCardTask",
		attr_bits: NULL,   ///< attribute bits
		cb_mem:NULL,   ///< memory for control block
		cb_size:NULL,   ///< size of provided memory for control block
		stack_mem:NULL,   ///< memory for stack
		stack_size : 1024 * 4,
		priority : (osPriority_t) osPriorityNormal,
};

osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
		.name = "myQueue01"
};

osMessageQueueId_t buzzerQueueHandle;
const osMessageQueueAttr_t buzzerQueue_attributes = {
		.name = "buzzerQueue"
};
//osQueue<buzzerStruct> (1,"buzzerQueue");
//osQueue<uint16_t> dysplayQueueHandle(16,"dysplayQueue");
osMessageQueueId_t toSDcardStringQueueHandle;
const osMessageQueueAttr_t toSDcardStringQueue_attributes = {
		.name = "toSDcardStringQueue"
};
//osQueue<Flash::stringStruct> toSDcardStringQueueHandle(1,"toSDcardStringQueue");

//osQueue<Keyboard::buttonEventStruct> keyboardQueueHandle(1,"keyboardQueue");
osMessageQueueId_t keyboardQueueHandle;
const osMessageQueueAttr_t keyboardQueue_attributes = {
		.name = "keyboardQueue"
};
osMessageQueueId_t GPS_UARTQueueHandle;
const osMessageQueueAttr_t GPS_UARTQueue_attributes = {
		.name = "GPS_UARTQueue"
};

//osQueue<uartNMEAstring> GPS_UARTQueueHandle(5,"GPS_UARTQueue");
/* Definitions for myBinarySem01 */
osEventFlagsId_t DataReadyEvent;



osMutexId_t I2C2MutexHandle;
const osMutexAttr_t I2C2Mutex_attributes = {
		.name = "I2C2Mutex",
};
osMutexId_t AccelStructMutexHandle;
const osMutexAttr_t AccelStructMutex_attributes = {
		.name = "AccelStructMutex",
};


/* USER CODE BEGIN PV */

TM_MPU9250_t accelStruct;
uint8_t receiveBuffer[32];

char stringBufferSDcard[96]={0};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
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



int main(void)
{
	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();

	BuzzerSetVolume(0);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

	//	HAL_Delay(50);
	////	ssd1306_Init();
	//	HAL_Delay(50);
	////	ssd1306_Fill(Black);
	//	HAL_Delay(50);
	////	ssd1306_UpdateScreen();
	//	HAL_Delay(50);



	// init();

	/* Init scheduler */
	osKernelInitialize();


	I2C2MutexHandle = osMutexNew(&I2C2Mutex_attributes);
	AccelStructMutexHandle = osMutexNew(&AccelStructMutex_attributes);
	//myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);
	DataReadyEvent = osEventFlagsNew(NULL);

	buzzerQueueHandle = osMessageQueueNew (1, sizeof(buzzerStruct), &buzzerQueue_attributes);
	keyboardQueueHandle = osMessageQueueNew(2, sizeof(Keyboard::buttonEventStruct), &keyboardQueue_attributes);
	GPS_UARTQueueHandle = osMessageQueueNew(8, sizeof(uint8_t)*128, &GPS_UARTQueue_attributes);
	toSDcardStringQueueHandle = osMessageQueueNew(24, sizeof(Flash::stringStruct), &toSDcardStringQueue_attributes);

	/* creation of defaultTask */
	//	defaultTask.start(StartDefaultTask);
	//	buzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &buzzerTask_attributes);
	//keyboardTaskHandle = osThreadNew(StartKeyboardTask, NULL, &keyboardTask_attributes);
	//	dysplayTaskHandle = osThreadNew(StartDysplayTask, NULL, &dysplayTask_attributes);

	accelTaskHandle = osThreadNew(StartAccelTask, NULL, &accelTask_attributes);
	gpsNMEA_ParserTHandle = osThreadNew(StartgpsNMEA_ParserTask, NULL, &gpsNMEA_ParserT_attributes);
	//sdCardTaskHandle = osThreadNew(StartSDcardTask, NULL, &sdCardTask_attributes);


	/* Start scheduler */
	osKernelStart();
	while (1)
	{

	}
}


void StartSDcardTask(void *argument)
{

	Flash::SDcard card;
	FRESULT init_result = card.initSD();

	osStatus_t queueStatus;
	Flash::stringStruct tempString;

	uint32_t syncTimer=osKernelGetTickCount();

	for(;;)
	{
		while(init_result!=FR_OK)
		{
			osDelay(10000);
			init_result = card.initSD();
			UART_Printf("SD ini\r\n");
			//buzzerQueueHandle.send(Flash::buzErrorInit);
		}

		queueStatus = osMessageQueueGet(toSDcardStringQueueHandle, &tempString, NULL, osWaitForever);   // wait for message   toSDcardStringQueueHandle.receive(&);
		if(queueStatus == osOK){
			FRESULT writeRes = card.writeString(tempString.string);
			if(writeRes!= FR_OK)
			{
				init_result =writeRes;
			}
		}
		if((osKernelGetTickCount()-syncTimer)>500)
		{
			syncTimer = osKernelGetTickCount();
			card.fileSync();
		}


	}

}




void StartDefaultTask(void *argument)
{
	for(;;)
	{
		osDelay(200);
	}
}

void StartBuzzerTask(void *argument)
{
	buzzerStruct buzzerParameters;
	osStatus_t status;
	for(;;)
	{
		status = osMessageQueueGet(buzzerQueueHandle, &buzzerParameters, NULL, 0U);   // wait for message   toSDcardStringQueueHandle.receive(&);

		if (status == osOK) {
			BuzzerSetFreq(buzzerParameters.freq);
			BuzzerSetVolume(buzzerParameters.volume);
			osDelay(buzzerParameters.duration);
			BuzzerSetVolume(0);
		}
	}
}
void StartKeyboardTask(void *argument)
{
	Keyboard::Hadler KeyboardHadler = Keyboard::Hadler(&keyboardQueueHandle,&buzzerQueueHandle);
	for(;;)
	{
		KeyboardHadler.checkKeyboard();
		osDelay(BTN_CHECK_DELAY);
	}
}
/* USER CODE END Header_StartDysplayTask */
void StartDysplayTask(void *argument)
{
	/* USER CODE BEGIN StartDysplayTask */
	/* Infinite loop */
	for(;;)
	{

		char str[32];
		//std::string ssa;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(2,2);
		sprintf(str, "Ax %d", accelStruct.Ax_Raw);
		ssd1306_WriteString(str,Font_7x10,White);

		ssd1306_SetCursor(2,12);
		sprintf(str, "Ay %d", accelStruct.Ay_Raw);
		ssd1306_WriteString(str,Font_7x10,White);

		ssd1306_SetCursor(2,22);
		sprintf(str, "Az %d", accelStruct.Az_Raw);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(60,2);
		sprintf(str, "Gx %d", accelStruct.Gx_Raw);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(60,12);
		sprintf(str, "Gy %d", accelStruct.Gy_Raw);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(60,22);
		sprintf(str, "Gz %d", accelStruct.Gz_Raw);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(2,33);
		sprintf(str, "Mx %d", accelStruct.Mx_Raw*50);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(2,43);
		sprintf(str, "My %d", accelStruct.My_Raw*50);
		ssd1306_WriteString(str,Font_7x10,White);
		ssd1306_SetCursor(2,53);
		sprintf(str, "Mz %d", accelStruct.Mz_Raw*50);
		ssd1306_WriteString(str,Font_7x10,White);

		//osStatus_t resultI2C = osMutexAcquire (I2C2MutexHandle, osWaitForever);
		//		if(resultI2C == osOK)
		//		{
		//			ssd1306_UpdateScreen();
		//		osMutexRelease(I2C2MutexHandle);
		//			}

		osDelay(400);
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
	uint32_t delay = osKernelGetTickCount();
	uint32_t Alldelay = osKernelGetTickCount();
	uint32_t  stop=0;
	uint32_t countMessage=0;
	buzzerStruct errQueue = {
			4000,
			700,
			BUZZER_VOLUME_MAX
	};
	TM_MPU9250_Result_t result;

	result = MPU9250_Init(&hi2c1,&accelStruct,TM_MPU9250_Device_0);
	UART_Printf("start accel task\r\n");

	for(;;)
	{
		Alldelay= osKernelGetTickCount();
		if(osEventFlagsWait(DataReadyEvent, 0x01, osFlagsWaitAny, 100)){
			//UART_Printf("%d",HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin));
			delay = osKernelGetTickCount();
			result =	TM_MPU9250_ReadAcce(&accelStruct);
			//result =	TM_MPU9250_ReadGyro(&accelStruct);
			//result =	TM_MPU9250_ReadMag(&accelStruct);
			if(countMessage%100==0){
				stop = osKernelGetTickCount()-delay;
//				UART_Printf("a %d:%d\r\n",stop,accelStruct.Ax_Raw);
			}
		}
		if(osEventFlagsWait(DataReadyEvent, 0x01, osFlagsWaitAny, 100)){
			//UART_Printf("%d",HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin));
			delay = osKernelGetTickCount();
			//result =	TM_MPU9250_ReadAcce(&accelStruct);
			result =	TM_MPU9250_ReadGyro(&accelStruct);
			result =	TM_MPU9250_ReadMag(&accelStruct);
			if(countMessage%100==0){
				stop = osKernelGetTickCount()-delay;
		//			UART_Printf("g %d:%d\r\n",stop,accelStruct.Gx_Raw);
			}
		}
		if(osEventFlagsWait(DataReadyEvent, 0x01, osFlagsWaitAny, 100)){
			//UART_Printf("%d",HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin));
			delay = osKernelGetTickCount();
			//result =	TM_MPU9250_ReadAcce(&accelStruct);
			//result =	TM_MPU9250_ReadGyro(&accelStruct);
			result =	TM_MPU9250_ReadMag(&accelStruct);
			if(countMessage%100==0){
				stop = osKernelGetTickCount()-delay;
		//		UART_Printf("m %d:%d\r\n",stop,accelStruct.Mx_Raw);
			}
		}
		if(countMessage%100==0){
			stop = osKernelGetTickCount()-Alldelay;

			UART_Printf("del:%d:%d\r\n",stop,accelStruct.Ax_Raw);}
		countMessage++;
		//osDelay(200);
	}
}

void StartgpsNMEA_ParserTask(void *argument)
{
	/* USER CODE BEGIN StartgpsNMEA_ParserTask */
	/* Infinite loop */
	USART1->CR1 |= USART_CR1_RXNEIE; /*//прерывание по приему данных*/
	HAL_UART_Receive_IT (&huart1, receiveBuffer, (uint8_t) 1);

	osStatus_t status;

	NMEA_UART::GPS_MESSEGE_TYPE messageType;
	NMEA_UART::Parser gpsParser;
	buzzerStruct errQueue = {
			4500,
			500,
			BUZZER_VOLUME_MAX
	};
	char buff[128];
	for(;;)
	{
		uint32_t start = osKernelGetTickCount();
		using namespace NMEA_UART;
		uint32_t countQ = osMessageQueueGetCount(GPS_UARTQueueHandle);
		if( countQ >= 7 )
		{
		//	osMessageQueuePut(buzzerQueueHandle, &errQueue, 0, 1);
		}

		status = osMessageQueueGet(GPS_UARTQueueHandle, &buff, NULL,osWaitForever );   // wait for message   toSDcardStringQueueHandle.receive(&);
		if (status == osOK) {
			messageType  = gpsParser.parseSting(buff);

			Flash::stringStruct tempString;
			memcpy(tempString.string,buff,strlen(buff));
			osStatus_t queueStat = osMessageQueuePut(toSDcardStringQueueHandle, &tempString, 0U, 0U);


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
}

void Error_Handler(void)
{
	HAL_UART_Transmit(&huart3, (uint8_t *) "ERROR ERROR!!!", 14, 10);
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
	//	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	//	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	//	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	//	{
	//		Error_Handler();
	//	}
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
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

#ifdef DEBUG_FROM_UART3

void UART_Printf(const char* fmt, ...) {
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&huart3, (uint8_t*)buff, strlen(buff),
			HAL_MAX_DELAY);
	va_end(args);
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
