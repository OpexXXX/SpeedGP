#ifndef SD_H_
#define SD_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
//--------------------------------------------------

#define SS_SD_SELECT() HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_SET)

//--------------------------------------------------
/* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */
//--------------------------------------------------

// Definitions for MMC/SDC command
/* MMC/SD command */
//#define CMD0	(0)			/* GO_IDLE_STATE */
//#define CMD1	(1)			/* SEND_OP_COND (MMC) */
//#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
//#define CMD8	(8)			/* SEND_IF_COND */
//#define CMD9	(9)			/* SEND_CSD */
//#define CMD10	(10)		/* SEND_CID */
//#define CMD12	(12)		/* STOP_TRANSMISSION */
//#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
//#define CMD16	(16)		/* SET_BLOCKLEN */
//#define CMD17	(17)		/* READ_SINGLE_BLOCK */
//#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
//#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
//#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
//#define CMD24	(24)		/* WRITE_BLOCK */
//#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
//#define CMD32	(32)		/* ERASE_ER_BLK_START */
//#define CMD33	(33)		/* ERASE_ER_BLK_END */
//#define CMD38	(38)		/* ERASE */
//#define CMD55	(55)		/* APP_CMD */
//#define CMD58	(58)		/* READ_OCR */

#define CMD0 (0x40+0) // GO_IDLE_STATE
#define CMD1 (0x40+1) // SEND_OP_COND (MMC)
#define ACMD41 (0xC0+41) // SEND_OP_COND (SDC)
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD8 (0x40+8) // SEND_IF_COND
#define CMD9 (0x40+9) // SEND_CSD
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR
typedef struct sd_info {
  volatile uint8_t type;//��� �����
} sd_info_ptr;
//--------------------------------------------------
void SD_PowerOn(void);
uint8_t sd_ini(void);
void SPI_Release(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_wait_ready(void);
uint8_t SD_cmd (uint8_t cmd, uint32_t arg);
//--------------------------------------------------
#endif /* SD_H_ */
