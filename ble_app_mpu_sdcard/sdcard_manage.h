#ifndef SDCARD_MANAGE_H__
#define SDCARD_MANAGE_H
#include "ff.h"
#include "diskio.h"
#include "integer.h"
#include "fattime.h"
#include "mpu_config.h"

#define LIST_FILE_NAME				"Record.txt"
#define REC_RQ_START					0x00
#define REC_RQ_FILENAME				0x01
#define REC_RQ_TIME						0x02
#define REC_ACK_START					0x10
#define REC_ACK_FILENAME			0x11
#define REC_ACK_TIME					0x12
#define REC_END								0x0F

typedef struct sdcard_fileio_s sdcard_fileio_t;

typedef struct sdcard_fileio_s
{
		FATFS *fs_getfree;			
		FATFS fs;            	// Work area (file system object) for logical drive
		FIL fsrc, fdst;      	// file objects
		UINT br, bw;         	// File R/W count
		DWORD j32;
		bool is_fs_open;			// Has the application already open a file io
		char * current_file;  // name of file being record currently
	} sdcard_fileio_t;

bool open_file_w(char* filename);
	
//Function for initializing SD card file system
bool sdcard_fs_init(sdcard_fileio_t * p_sd_fileio, mpu_config_t * p_config);

//Function for starting recording MPU measurements
bool start_record(char * filename);

//Function for stopping recording
bool stop_record(void);

//Get and record one measurement
bool record(void);
		
#endif
