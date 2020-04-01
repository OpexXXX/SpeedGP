/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include <sdlowlevel.h>
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include <stdio.h>
#include <string.h>
/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

extern UART_HandleTypeDef huart3;
extern char str1[60];
extern sd_info_ptr sdinfo;
extern SPI_HandleTypeDef hspi2;


static volatile DSTATUS Stat = STA_NOINIT;

DSTATUS disk_status (
		BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
#ifdef SD_DEBUG
	sprintf(str1,"disk.cpp disk_status() \r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
		BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	Stat &= ~STA_NOINIT;
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
		BYTE pdrv,		/* Physical drive nmuber to identify the drive */
		BYTE *buff,		/* Data buffer to store read data */
		LBA_t sector,	/* Start sector in LBA */
		UINT count		/* Number of sectors to read */
)
{
	/* USER CODE BEGIN READ */
#ifdef SD_DEBUG
	sprintf(str1,"disk.cpp disk_read()  sector: %lu; count: %d\r\n",sector, count);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif

	if (pdrv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	 sector *= 512; /* Convert to byte address if needed  if (!(sdinfo.type & 4)) */

	if (count == 1) /* Single block read */
	{
		uint8_t res  =  SD_Read_Block(buff,sector);
		if(res!=0) return RES_ERROR;
		count = 0;
	}
	else /* Multiple block read */
	{

		HAL_UART_Transmit(&huart3,(uint8_t*)"Multiple R\r\n",12,0x1000);

	}
	SPI_Release();
	return count ? RES_ERROR : RES_OK;
	/* USER CODE END READ */
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
		BYTE pdrv,			/* Physical drive nmuber to identify the drive */
		const BYTE *buff,	/* Data to be written */
		LBA_t sector,		/* Start sector in LBA */
		UINT count			/* Number of sectors to write */
)
{
#ifdef SD_DEBUG
	sprintf(str1,"disk.cpp disk_write()  sector: %lu; count: %d\r\n",sector, count);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif

	if (pdrv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

		sector *= 512;
	 /* Convert to byte address if needed */
	if (count == 1) /* Single block read */
	{
		uint8_t res = SD_Write_Block((BYTE*)buff,sector); //Ñ÷èòàåì áëîê â áóôåð
		if (res!=0)return RES_ERROR;
		count = 0;
	}
	else /* Multiple block read */
	{
		//	HAL_UART_Transmit(&huart3,(uint8_t*)"Multiple W\r\n",12,0x1000);
	}
	SPI_Release();
	return count ? RES_ERROR : RES_OK;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
		BYTE pdrv,		/* Physical drive nmuber (0..) */
		BYTE cmd,		/* Control code */
		void *buff		/* Buffer to send/receive control data */
)
{

	DRESULT res;

#ifdef SD_DEBUG
	sprintf(str1,"disk.cpp disk_ioctl()  CMD: %d\r\n",cmd);
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),0x1000);
#endif
	if (pdrv) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	res = RES_ERROR;
	switch (cmd)
	{
	case CTRL_SYNC : /* Flush dirty buffer if present */
		SS_SD_SELECT();
		if (SPI_wait_ready() == 0xFF)
			res = RES_OK;
		break;
	case GET_SECTOR_SIZE : /* Get sectors on the disk (WORD) */
		*(WORD*)buff = 512;
		res = RES_OK;
		break;


	default:
		res = RES_PARERR;
	}
	SPI_Release();
	return res;
}

