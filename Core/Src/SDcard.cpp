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
FRESULT SDcard::fileSync()
{
	FRESULT res;
		res = f_sync(&file);
		if(res != FR_OK) {
	#ifdef DEBUG_SDCARD_CPP
			UART_Printf("f_sync() failed, res = %d\r\n", res);
	#endif
			return res;
		}
		return FR_OK;
}
FRESULT SDcard::openFile()
{
	FRESULT res;
	res = f_open(&file,"accel.txt", FA_OPEN_APPEND | FA_WRITE);
	if(res != FR_OK) {
#ifdef DEBUG_SDCARD_CPP
		UART_Printf("f_open() failed, res = %d\r\n", res);
#endif
		return res;
	}
	return FR_OK;
}
FRESULT SDcard::writeString(const char * strin )
{
	FRESULT res;
	int bytesWritten=0;
	bytesWritten = f_printf(&file,"%s\n", strin);//(&logFile, strin, bytesToWrite, &bytesWritten);
#ifdef DEBUG_SDCARD_CPP
		UART_Printf("f_printf() %d bytes \r\n",bytesWritten);
#endif
	if(bytesWritten == -1) {
#ifdef DEBUG_SDCARD_CPP
		UART_Printf("f_puts() failed, res = %d\r\n", res);
#endif
		res = FR_DISK_ERR;
		return res;
	}
//
//#ifdef DEBUG_SDCARD_CPP
////	if(bytesWritten < bytesToWrite) {
////		UART_Printf("WARNING! Disk is full.\r\n");
////	}
//
//		UART_Printf("f_close() \r\n");
//#endif
//	res = f_close(&logFile);
//	if(res != FR_OK) {
//#ifdef DEBUG_SDCARD_CPP
//		UART_Printf("f_close() failed, res = %d\r\n", res);
//#endif
//		return res;
//	}
	return FR_OK;
}

FRESULT SDcard::initSD(){
#ifdef DEBUG_SDCARD_CPP
		UART_Printf("Init SD \r\n");
#endif
	init = sd_ini()?FR_DISK_ERR:FR_OK;
	if(init==0){
		mount = f_mount(&fs, "", 0);
		if(mount == FR_OK){
			mount = openFile();
			return mount;
		}
		return mount;

	}else{
		return FR_DISK_ERR;
	}
}


}
