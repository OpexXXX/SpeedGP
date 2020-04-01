/*
 * SDcard.cpp
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: 79029
 */

#include <SDcard.h>
namespace Flash
{
SDcard::SDcard() {
	// TODO Автоматически созданная заглушка конструктора

}

SDcard::~SDcard() {
	// TODO !CodeTemplates.destructorstub.tododesc!
}
FRESULT SDcard::writeString(const char * filename,const char * strin )
{
	FRESULT res;
	FIL logFile;
#ifdef DEBUG_FROM_UART3
		UART_Printf("f_open() \r\n");
#endif
	res = f_open(&logFile,filename, FA_OPEN_APPEND | FA_WRITE);
	if(res != FR_OK) {
#ifdef DEBUG_FROM_UART3
		UART_Printf("f_open() failed, res = %d\r\n", res);
#endif
		return res;
	}
#ifdef DEBUG_FROM_UART3
	UART_Printf("f_open() OK %d" , res);
#endif
	//unsigned int bytesToWrite = strlen(strin);
	int bytesWritten=0;
	bytesWritten = f_printf(&logFile,"%s\n", strin);//(&logFile, strin, bytesToWrite, &bytesWritten);
#ifdef DEBUG_FROM_UART3
		UART_Printf("f_printf() %d bytes \r\n",bytesWritten);
#endif
	if(bytesWritten == -1) {
#ifdef DEBUG_FROM_UART3
		UART_Printf("f_puts() failed, res = %d\r\n", res);
#endif
		res = FR_DISK_ERR;
		return res;
	}

#ifdef DEBUG_FROM_UART3
//	if(bytesWritten < bytesToWrite) {
//		UART_Printf("WARNING! Disk is full.\r\n");
//	}

		UART_Printf("f_close() \r\n");
#endif
	res = f_close(&logFile);
	if(res != FR_OK) {
#ifdef DEBUG_FROM_UART3
		UART_Printf("f_close() failed, res = %d\r\n", res);
#endif
		return res;
	}
	return FR_OK;
}

FRESULT SDcard::initSD(){
#ifdef DEBUG_FROM_UART3
		UART_Printf("Init SD \r\n");
#endif
	init = sd_ini()?FR_DISK_ERR:FR_OK;
	if(init==0){
		mount = f_mount(&fs, "", 0);
		return mount;
	}else{
		return FR_DISK_ERR;
	}
}


}
