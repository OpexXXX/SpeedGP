/*
 * SDcard.h
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: 79029
 */

#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_
#include "defines.h"
#include <sdlowlevel.h>
#include "ff.h"
#include "Buzzer.h"

namespace Flash {



 struct stringStruct {
const char* fileName;
char string[100];
};


class SDcard {
private:
	FATFS fs;
	FRESULT init;
	FRESULT mount;
	uint32_t totalBlocks;
	uint32_t freeBlocks;
	FIL file;
public:
	FRESULT openFile();
	FRESULT fileSync();
	SDcard();
	virtual ~SDcard();
	FRESULT initSD();
	FRESULT writeString(const char * strin );
	uint32_t getFreeBlocks();
	uint32_t gettotalBlocks();
};

//
//void init() {
//	FATFS fs;
//
//#ifdef DEBUG_FROM_UART3
//	UART_Printf("Ready!\r\n");
//#endif
//	// mount the default drive
//	res = f_mount(&fs, "", 0);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("f_mount() failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//#ifdef DEBUG_FROM_UART3
//	UART_Printf("f_mount() done!\r\n");
//#endif
//	uint32_t freeClust;
//	FATFS* fs_ptr = &fs;
//	// Warning! This fills fs.n_fatent and fs.csize!
//	res = f_getfree("/", &freeClust, &fs_ptr);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("f_getfree() failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//#ifdef DEBUG_FROM_UART3
//	UART_Printf("f_getfree() done!\r\n");
//#endif
//
//	uint32_t totalBlocks = (fs.n_fatent - 2) * fs.csize;
//	uint32_t freeBlocks = freeClust * fs.csize;
//
//
//#ifdef DEBUG_FROM_UART3
//	switch (fs.fs_type) {//(0, FS_FAT12, FS_FAT16, FS_FAT32 or FS_EXFAT) */
//	case 1:
//		UART_Printf("FAT12\r\n");
//		break;
//	case 2:
//		UART_Printf( "FAT16\r\n");
//		break;
//	case 3:
//		UART_Printf( "FAT32\r\n");
//		break;
//	case 4:
//		UART_Printf("EXFAT\r\n");
//		break;
//	default:
//		break;
//	}
//
//	UART_Printf("Total blocks: %lu (%lu Mb)\r\n",
//			totalBlocks, totalBlocks / 2000);
//	UART_Printf("Free blocks: %lu (%lu Mb)\r\n",
//			freeBlocks, freeBlocks / 2000);
//
//
//	DIR dir;
//	res = f_opendir(&dir, "/");
//	if(res != FR_OK) {
//
//
//		return;
//	}
//
//	FILINFO fileInfo;
//	uint32_t totalFiles = 0;
//	uint32_t totalDirs = 0;
//
//	for(;;) {
//		res = f_readdir(&dir, &fileInfo);
//		if((res != FR_OK) || (fileInfo.fname[0] == '\0')) {
//			break;
//		}
//
//		if(fileInfo.fattrib & AM_DIR) {
//			UART_Printf("  DIR  %s\r\n", fileInfo.fname);
//			totalDirs++;
//		} else {
//			UART_Printf("  FILE %s\r\n", fileInfo.fname);
//			totalFiles++;
//		}
//	}
//
//	UART_Printf("(total: %lu dirs, %lu files)\r\n--------\r\n",
//			totalDirs, totalFiles);
//
//	res = f_closedir(&dir);
//	if(res != FR_OK) {
//		UART_Printf("f_closedir() failed, res = %d\r\n", res);
//		return;
//	}
//
//	UART_Printf("Writing to log1.txt...\r\n");
//#endif
//
//	char writeBuff[128];
//
//	snprintf(writeBuff, sizeof(writeBuff),
//			"Total blocks: %lu (%lu Mb); Free blocks: %lu (%lu Mb)\r\n",
//			totalBlocks, totalBlocks / 2000,
//			freeBlocks, freeBlocks / 2000);
//
//	FIL logFile;
//	res = f_open(&logFile, "log1.txt", FA_OPEN_APPEND | FA_WRITE);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("f_open() failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//
//	unsigned int bytesToWrite = strlen(writeBuff);
//	unsigned int bytesWritten;
//	res = f_write(&logFile, writeBuff, bytesToWrite, &bytesWritten);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("f_write() failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//
//#ifdef DEBUG_FROM_UART3
//	if(bytesWritten < bytesToWrite) {
//		UART_Printf("WARNING! Disk is full.\r\n");
//	}
//#endif
//
//	res = f_close(&logFile);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("f_close() failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//
//#ifdef DEBUG_FROM_UART3
//	UART_Printf("Reading file...\r\n");
//	FIL msgFile;
//	res = f_open(&msgFile, "log1.txt", FA_READ);
//	if(res != FR_OK) {
//		UART_Printf("f_open() failed, res = %d\r\n", res);
//		return;
//	}
//
//	char readBuff[128];
//	unsigned int bytesRead;
//	res = f_read(&msgFile, readBuff, sizeof(readBuff)-1, &bytesRead);
//	if(res != FR_OK) {
//		UART_Printf("f_read() failed, res = %d\r\n", res);
//		return;
//	}
//
//	readBuff[bytesRead] = '\0';
//	UART_Printf("```\r\n%s\r\n```\r\n", readBuff);
//
//	res = f_close(&msgFile);
//	if(res != FR_OK) {
//		UART_Printf("f_close() failed, res = %d\r\n", res);
//		return;
//	}
//
//#endif
//	// Unmount
//	res = f_mount(NULL, "", 0);
//	if(res != FR_OK) {
//#ifdef DEBUG_FROM_UART3
//		UART_Printf("Unmount failed, res = %d\r\n", res);
//#endif
//		return;
//	}
//
//#ifdef DEBUG_FROM_UART3
//	UART_Printf("Done!\r\n");
//#endif
//}
//
}  // namespace Flash

#endif /* INC_SDCARD_H_ */
