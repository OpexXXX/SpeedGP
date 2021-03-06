#include <sdlowlevel.h>
#ifdef SD_DEBUG
#include <stdio.h>
#include <string.h>
#endif
//--------------------------------------------------

//--------------------------------------------------
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;

sd_info_ptr sdinfo;
char str1[60]={0};
//--------------------------------------------------
static void Error (void)
{

}
//-----------------------------------------------
uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		Error();
	}
	return receivedbyte;
}
//-----------------------------------------------
void SPI_SendByte(uint8_t bt)
{

	SPIx_WriteRead(bt);
}
//-----------------------------------------------
uint8_t SPI_ReceiveByte(void)
{
	uint8_t bt = SPIx_WriteRead(0xFF);
	return bt;
}
//-----------------------------------------------
void SPI_Release(void)
{
	SPIx_WriteRead(0xFF);
}
//-----------------------------------------------
uint8_t SPI_wait_ready(void)
{
	uint8_t res;
	uint16_t cnt;
	cnt=0;
	do { //���� ��������� ��������� BUSY
		res=SPI_ReceiveByte();
		cnt++;
	} while ( (res!=0xFF)&&(cnt<0xFFFF) );
	if (cnt>=0xFFFF) return 1;
	return res;
}
//-----------------------------------------------
uint8_t SD_cmd (uint8_t cmd, uint32_t arg)
{
#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_cmd :%d  value:0x%04X\r\n",cmd,arg);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	uint8_t n, res;
	// ACMD<n> is the command sequense of CMD55-CMD<n>
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		res = SD_cmd(CMD55, 0);
		if (res > 1) return res;
	}
	// Select the card
	SS_SD_DESELECT();
	SPI_ReceiveByte();
	SS_SD_SELECT();
	SPI_ReceiveByte();
	// Send a command packet
	SPI_SendByte(cmd); // Start + Command index
	SPI_SendByte((uint8_t)(arg >> 24)); // Argument[31..24]
	SPI_SendByte((uint8_t)(arg >> 16)); // Argument[23..16]
	SPI_SendByte((uint8_t)(arg >> 8)); // Argument[15..8]
	SPI_SendByte((uint8_t)arg); // Argument[7..0]
	n = 0x01; // Dummy CRC + Stop
	if (cmd == CMD0) {n = 0x95;} // Valid CRC for CMD0(0)
	if (cmd == CMD8) {n = 0x87;} // Valid CRC for CMD8(0x1AA)
	SPI_SendByte(n);	
	// Receive a command response
	n = 10; // Wait for a valid response in timeout of 10 attempts
	do {
		res = SPI_ReceiveByte();
	} while ((res & 0x80) && --n);
	return res;
}
//-----------------------------------------------
void SD_PowerOn(void)
{
	HAL_Delay(30);
}
//-----------------------------------------------
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba)
{
#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_Read_Block: 0x%010X  %d\r\n",lba,lba);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	uint8_t result;
	uint16_t cnt;

	result=SD_cmd (CMD17, lba); //CMD17 ������� ��� 50 � 96
	if (result!=0x00) {

#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_Read_Block: Error 5\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		return 5;
	} //�����, ���� ��������� �� 0x00
	SPI_Release();
	cnt=0;
	do{ //���� ������ �����
		result=SPI_ReceiveByte();
		cnt++;
	} while ( (result!=0xFE)&&(cnt<0xFFFF) );
	if (cnt>=0xFFFF)
	{

#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_Read_Block: Error 5\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		return 5;
	}

	for (cnt=0;cnt<512;cnt++) buff[cnt]=SPI_ReceiveByte(); //�������� ����� ����� �� ���� � �����
	SPI_Release(); //���������� ����������� �����
	SPI_Release();
#ifdef SD_DEBUG_SHOW_READ
	sprintf(str1,"     reading data:\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);

	for (cnt=0;cnt<16;cnt++)
	{
		sprintf(str1,"0x%02X ",cnt);
			HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
	}

	HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,0x1000);
	HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,0x1000);
	for (cnt=0;cnt<512;cnt++){

	if(cnt%16==0&&cnt!=0)HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,0x1000);
	sprintf(str1,"0x%02X ", buff[cnt]);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
	}
	HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,0x1000);
	HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,0x1000);

#endif
	return 0;
}
//-----------------------------------------------
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba)
{

#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_Write_Block: 0x%010X  %d\r\n",lba,lba);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif

	uint8_t result;
	uint16_t cnt;
	result=SD_cmd(CMD24,lba);
	if (result!=0x00){

#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_WRITE_Block: Error 6 \r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		return 6;
	}
	SPI_Release();
	SPI_SendByte (0xFE);
	for (cnt=0;cnt<512;cnt++) SPI_SendByte(buff[cnt]);
	SPI_Release();
	SPI_Release();
	result=SPI_ReceiveByte();
	if ((result&0x05)!=0x05){

#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_WRITE_Block: Error 6 \r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		return 6;

	}
	cnt=0;
	do { //
		result=SPI_ReceiveByte();
		cnt++;
	} while ( (result!=0xFF)&&(cnt<0xFFFF) );
	if (cnt>=0xFFFF){
#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp SD_WRITE_Block: Error 6 \r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
		return 6;
	}
	return 0;
}
//-----------------------------------------------
uint8_t sd_ini(void)
{
#ifdef SD_DEBUG
	sprintf(str1,"sd.cpp sd_ini\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	uint8_t i, cmd;
	int16_t tmr;
	uint32_t temp;


	sdinfo.type = 0;
	uint8_t ocr[4];

	temp = hspi2.Init.BaudRatePrescaler;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //156.25 kbbs
	HAL_SPI_Init(&hspi2);

	SS_SD_DESELECT();
	for(i=0;i<10;i++) //80 ��������� (�� ����� 74) ������� ��� 91
		SPI_Release();
	hspi2.Init.BaudRatePrescaler = temp;
	HAL_SPI_Init(&hspi2);
	SS_SD_SELECT();

	if (SD_cmd(CMD0, 0) == 1) // Enter Idle state
	{
		HAL_Delay(1000);
		//SPI_Release();
		if (SD_cmd(CMD8, 0x1AA) == 1) // SDv2
		{
			for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
#ifdef SD_DEBUG
			sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",ocr[0],ocr[1],ocr[2],ocr[3]);
			HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
			// Get trailing return value of R7 resp
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) // The card can work at vdd range of 2.7-3.6V
			{
				for (tmr = 12000; tmr && SD_cmd(ACMD41, 1UL << 30); tmr--)
					; // Wait for leaving idle state (ACMD41 with HCS bit)
				if (tmr && SD_cmd(CMD58, 0) == 0)
				{ // Check CCS bit in the OCR
					for (i = 0; i < 4; i++)
						ocr[i] = SPI_ReceiveByte();
#ifdef SD_DEBUG
					sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",ocr[0],ocr[1],ocr[2],ocr[3]);
					HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
					sdinfo.type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)

				}
			}
		}
		else //SDv1 or MMCv3
		{
			if (SD_cmd(ACMD41, 0) <= 1)
			{
				sdinfo.type = CT_SD1; cmd = ACMD41; // SDv1
			}
			else
			{
				sdinfo.type = CT_MMC; cmd = CMD1; // MMCv3
			}
			for (tmr = 25000; tmr && SD_cmd(cmd, 0); tmr--) ; // Wait for leaving idle state
			if (!tmr || SD_cmd(CMD16, 512) != 0) // Set R/W block length to 512
				sdinfo.type = 0;

		}

	}
	else
	{
		return 1;
	}
#ifdef SD_DEBUG
	sprintf(str1,"Type SD: 0x%02X\r\n",sdinfo.type);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	return 0;
}
//-----------------------------------------------
