
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Buzzer.h"
#include "ssd1306.h"
#include "fonts.h"
#include "keyboard.h"
#include "accelerometer.h"
#include "helper.h"
#include "nmea_parser.h"
#include "sd.h"

#include "ff.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osTask defaultTask("defaultTask",64);
osTask buzzerTask("buzzerTask",64);
osTask keyboardTask("keyboardTask",70);
osTask dysplayTask("dysplayTask",140);
osTask accelTask("accelTask",64);
osTask gpsNMEA_ParserT("gpsNMEA_ParserT",200);
osTask sdCardTask("sdCardTask",1024,osPriorityAboveNormal3);

osQueue<buzzerStruct> buzzerQueueHandle(2,"buzzerQueue");
osQueue<uint16_t> dysplayQueueHandle(16,"dysplayQueue");
osQueue<Keyboard::buttonEventStruct> keyboardQueueHandle(1,"keyboardQueue");
osQueue<uint8_t> GPS_UARTQueueHandle(64,"GPS_UARTQueue");

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
void UART_Printf(const char* fmt, ...);

void init() {
    FATFS fs;
    FRESULT res;
    UART_Printf("Ready!\r\n");

    // mount the default drive
    res = f_mount(&fs, "", 0);
    if(res != FR_OK) {
        UART_Printf("f_mount() failed, res = %d\r\n", res);
        return;
    }
    UART_Printf("f_mount() done!\r\n");
    uint32_t freeClust;
    FATFS* fs_ptr = &fs;
    // Warning! This fills fs.n_fatent and fs.csize!
    res = f_getfree("/", &freeClust, &fs_ptr);
    if(res != FR_OK) {
        UART_Printf("f_getfree() failed, res = %d\r\n", res);
        return;
    }

    UART_Printf("f_getfree() done!\r\n");
    uint32_t totalBlocks = (fs.n_fatent - 2) * fs.csize;
    uint32_t freeBlocks = freeClust * fs.csize;
    switch (fs.fs_type) {//(0, FS_FAT12, FS_FAT16, FS_FAT32 or FS_EXFAT) */
		case 1:
			 UART_Printf("FAT12\r\n");
			break;
		case 2:
			  UART_Printf( "FAT16\r\n");
					break;
		case 3:
			  UART_Printf( "FAT32\r\n");
					break;
		case 4:
			  UART_Printf("EXFAT\r\n");
					break;
		default:
			break;
	}

    UART_Printf("Total blocks: %lu (%lu Mb)\r\n",
                totalBlocks, totalBlocks / 2000);
    UART_Printf("Free blocks: %lu (%lu Mb)\r\n",
                freeBlocks, freeBlocks / 2000);

    DIR dir;
    res = f_opendir(&dir, "/");
    if(res != FR_OK) {
        UART_Printf("f_opendir() failed, res = %d\r\n", res);
        return;
    }

    FILINFO fileInfo;
    uint32_t totalFiles = 0;
    uint32_t totalDirs = 0;
    UART_Printf("--------\r\nRoot directory:\r\n");
    for(;;) {
        res = f_readdir(&dir, &fileInfo);
        if((res != FR_OK) || (fileInfo.fname[0] == '\0')) {
            break;
        }

        if(fileInfo.fattrib & AM_DIR) {
            UART_Printf("  DIR  %s\r\n", fileInfo.fname);
            totalDirs++;
        } else {
            UART_Printf("  FILE %s\r\n", fileInfo.fname);
            totalFiles++;
        }
    }

    UART_Printf("(total: %lu dirs, %lu files)\r\n--------\r\n",
                totalDirs, totalFiles);

    res = f_closedir(&dir);
    if(res != FR_OK) {
        UART_Printf("f_closedir() failed, res = %d\r\n", res);
        return;
    }

    UART_Printf("Writing to log1.txt...\r\n");

    char writeBuff[128];
    snprintf(writeBuff, sizeof(writeBuff),
        "Total blocks: %lu (%lu Mb); Free blocks: %lu (%lu Mb)\r\n",
        totalBlocks, totalBlocks / 2000,
        freeBlocks, freeBlocks / 2000);

    FIL logFile;
    res = f_open(&logFile, "log1.txt", FA_OPEN_APPEND | FA_WRITE);
    if(res != FR_OK) {
        UART_Printf("f_open() failed, res = %d\r\n", res);
        return;
    }

    unsigned int bytesToWrite = strlen(writeBuff);
    unsigned int bytesWritten;
    res = f_write(&logFile, writeBuff, bytesToWrite, &bytesWritten);
    if(res != FR_OK) {
        UART_Printf("f_write() failed, res = %d\r\n", res);
        return;
    }

    if(bytesWritten < bytesToWrite) {
        UART_Printf("WARNING! Disk is full.\r\n");
    }

    res = f_close(&logFile);
    if(res != FR_OK) {
        UART_Printf("f_close() failed, res = %d\r\n", res);
        return;
    }

    UART_Printf("Reading file...\r\n");
    FIL msgFile;
    res = f_open(&msgFile, "log1.txt", FA_READ);
    if(res != FR_OK) {
        UART_Printf("f_open() failed, res = %d\r\n", res);
        return;
    }

    char readBuff[128];
    unsigned int bytesRead;
    res = f_read(&msgFile, readBuff, sizeof(readBuff)-1, &bytesRead);
    if(res != FR_OK) {
        UART_Printf("f_read() failed, res = %d\r\n", res);
        return;
    }

    readBuff[bytesRead] = '\0';
    UART_Printf("```\r\n%s\r\n```\r\n", readBuff);

    res = f_close(&msgFile);
    if(res != FR_OK) {
        UART_Printf("f_close() failed, res = %d\r\n", res);
        return;
    }

    // Unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK) {
        UART_Printf("Unmount failed, res = %d\r\n", res);
        return;
    }

    UART_Printf("Done!\r\n");
}

int main(void)
{
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

	MPU9250_Init(&hi2c1,&accelStruct,TM_MPU9250_Device_0);


	USART1->CR1 |= USART_CR1_RXNEIE; /*//прерывание по приему данных*/
	HAL_UART_Receive_IT (&huart1, receiveBuffer, (uint8_t) 1);

    sd_ini();
   // init();


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

void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart3, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
    va_end(args);
}


void StartSDcardTask(void *argument)
{
	   init();
	for(;;)
	{
		osDelay(20);
	}
}




void StartDefaultTask(void *argument)
{
	for(;;)
	{
		osDelay(20);
	}
}

void StartBuzzerTask(void *argument)
{
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
