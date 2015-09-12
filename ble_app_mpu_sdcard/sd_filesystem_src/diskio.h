/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file  R0.07   (C)ChaN, 2009
/-----------------------------------------------------------------------*/

#ifndef _DISKIO

#include "integer.h"
#include "spi_master.h"




// demo uses a command line option to define this (see Makefile):
// #define STM32_SD_USE_DMA

/* set to 1 to provide a disk_ioctrl function even if not needed by the FatFs */
#define STM32_SD_DISK_IOCTRL_FORCE      1



/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card-Select Controls  (Platform dependent) */
#define SELECT()        enable_ss()    /* MMC CS = L */
#define DESELECT()      disable_ss()      /* MMC CS = H */

/* Manley EK-STM32F board does not offer socket contacts -> dummy values: */
#define SOCKPORT	1			/* Socket contact port */
#define SOCKWP		0			/* Write protect switch (PB5) */
#define SOCKINS		0			/* Card detect switch (PB4) */

#if (_MAX_SS != 512) || (_FS_READONLY == 0) || (STM32_SD_DISK_IOCTRL_FORCE == 1)
#define STM32_SD_DISK_IOCTRL   1
#else
#define STM32_SD_DISK_IOCTRL   0
#endif








/* Status of Disk Functions */
typedef BYTE DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,     /* 0: Successful */
	RES_ERROR,      /* 1: R/W Error */
	RES_WRPRT,      /* 2: Write Protected */
	RES_NOTRDY,     /* 3: Not Ready */
	RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

BOOL assign_drives (int argc, char *argv[]);
DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);
DRESULT disk_read (BYTE, BYTE*, DWORD, BYTE);
#if _READONLY == 0
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
#endif
DRESULT disk_ioctl (BYTE, BYTE, void*);



/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT      0x01  /* Drive not initialized */
#define STA_NODISK      0x02  /* No medium in the drive */
#define STA_PROTECT     0x04  /* Write protected */


/* Command code for disk_ioctrl() */

/* Generic command */
#define CTRL_SYNC           0  /* Mandatory for write functions */
#define GET_SECTOR_COUNT    1  /* Mandatory for only f_mkfs() */
#define GET_SECTOR_SIZE     2
#define GET_BLOCK_SIZE      3  /* Mandatory for only f_mkfs() */
#define CTRL_POWER          4
#define CTRL_LOCK           5
#define CTRL_EJECT          6
/* MMC/SDC command */
#define MMC_GET_TYPE        10
#define MMC_GET_CSD         11
#define MMC_GET_CID         12
#define MMC_GET_OCR         13
#define MMC_GET_SDSTAT      14
/* ATA/CF command */
#define ATA_GET_REV         20
#define ATA_GET_MODEL       21
#define ATA_GET_SN          22


/* Martin Thomas begin */

/* Card type flags (CardType) */
#define CT_MMC              0x01
#define CT_SD1              0x02
#define CT_SD2              0x04
#define CT_SDC              (CT_SD1|CT_SD2)
#define CT_BLOCK            0x08

#ifndef RAMFUNC
#define RAMFUNC
#endif
RAMFUNC void disk_timerproc (void);

/* Martin Thomas end */

#define _DISKIO
#endif
