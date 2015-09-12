#include <stdlib.h>
#include "sdcard_manage.h"
#include "nrf_delay.h"
#include "ble_mps.h"

static mpu_config_t * p_mpu_config;
static sdcard_fileio_t * p_sdcard_fileio;
static uint8_t start_time[6];
static int data_num;

//Append string to current opened file
static bool appends_current(const void *buffer, uint8_t len)
{
		uint8_t buf[6];
		FRESULT res;         	// FatFs function common result code
		get_time(buf);
		res = f_lseek(&p_sdcard_fileio->fdst, p_sdcard_fileio->fdst.fsize);//Move to end of file
		res = f_printf(&p_sdcard_fileio->fdst, "%s %02d.%02d. %02d:%02d:%02d.%02d\n", buffer, buf[5], buf[4], buf[3], buf[2], buf[1], buf[0]);

		if(res!=FR_OK)
		{
			return false;
		}
		return true;
}

bool open_file_w(char* filename)
{
		FRESULT res;         	// FatFs function common result code
		res = f_open(&p_sdcard_fileio->fdst, filename, FA_OPEN_ALWAYS | FA_WRITE);
		if(res == FR_OK)
		{
			p_sdcard_fileio->is_fs_open = true;
			
			p_sdcard_fileio->current_file = (char *) malloc(sizeof(filename));
			memcpy(p_sdcard_fileio->current_file, filename, strlen(filename));
			return true;
		}
		return false;
}

void update_records_char(void)
{		
		char int_buf[4];
		UINT rb;
		f_open(&p_sdcard_fileio->fdst, LIST_FILE_NAME, FA_OPEN_ALWAYS | FA_READ);
		memset(int_buf, 0, sizeof(int_buf));
		f_read(&p_sdcard_fileio->fdst,int_buf, 4, &rb);
		set_mps_records(int_buf);
		f_close(&p_sdcard_fileio->fdst);
}

//Function for initializing SD card file system
bool sdcard_fs_init(sdcard_fileio_t * p_sd_fileio, mpu_config_t * p_config)
{
		FRESULT res;         	// FatFs function common result code
		
		p_sdcard_fileio = p_sd_fileio;
		p_mpu_config = p_config;
		data_num = 0;
	
		f_mount(0, &p_sdcard_fileio->fs);
		res = f_getfree("0:", &p_sdcard_fileio->j32, &p_sdcard_fileio->fs_getfree);
		if(res != FR_OK)
			return false;
		
		f_mkdir("MPU_DATA");
		f_chdir("/MPU_DATA");
		
		if(f_open(&p_sdcard_fileio->fdst,LIST_FILE_NAME, FA_CREATE_NEW) == FR_OK)
		{//File doesn't exist, a new file is created
				f_close(&p_sdcard_fileio->fdst);
				f_open(&p_sdcard_fileio->fdst, LIST_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE);
				f_printf(&p_sdcard_fileio->fdst, "%04d\n", 0);
		}
		f_close(&p_sdcard_fileio->fdst);
		
		update_records_char();

		return true;
}



bool start_record(char * filename)
{		
		char msg_buf[20];
		char int_buf[4];
		UINT rb;
	
		if(p_sdcard_fileio->is_fs_open)
		{//FS busy
			return false;
		}
		data_num = 0;
	
		/**Update record file*/		
		//update number of records
		f_open(&p_sdcard_fileio->fdst, LIST_FILE_NAME, FA_OPEN_ALWAYS | FA_READ);
		memset(int_buf, 0, sizeof(int_buf));
		f_read(&p_sdcard_fileio->fdst,int_buf, 4, &rb);
		f_close(&p_sdcard_fileio->fdst);

		//Append new file name
		open_file_w(LIST_FILE_NAME);
		int f_num = atoi(int_buf)+1;
		f_printf(&p_sdcard_fileio->fdst, "%04d", f_num);
		appends_current(filename, strlen(filename));
		f_close(&p_sdcard_fileio->fdst);
		
		open_file_w(filename);
		get_time(start_time);//Get start time
		get_mps_message(msg_buf);
		
		//Write header to file
		f_printf(&p_sdcard_fileio->fdst, "%06d data collected\n", 0);
		f_printf(&p_sdcard_fileio->fdst, "Start:%2d:%2d:%2d.%2d\n", 0, 0, 0, 0);
		f_printf(&p_sdcard_fileio->fdst, "End:%2d:%2d:%2d.%2d\n", 0, 0, 0, 0);
		f_printf(&p_sdcard_fileio->fdst, "ID:%s Types:", p_mpu_config->id);
		if(p_mpu_config->acc_enable)
			f_printf(&p_sdcard_fileio->fdst, " ACCEL");
		if(p_mpu_config->gyro_enable)
			f_printf(&p_sdcard_fileio->fdst, " GYRO");
		if(p_mpu_config->mag_enable)
			f_printf(&p_sdcard_fileio->fdst, " MAG");
		if(p_mpu_config->temp_enable)
			f_printf(&p_sdcard_fileio->fdst, " TEMP");
		
		f_printf(&p_sdcard_fileio->fdst, "\n");
		f_printf(&p_sdcard_fileio->fdst, "Scale:%2X/%2X Sampling rate:%d\n",p_mpu_config->acc_scale,p_mpu_config->gyro_scale, p_mpu_config->sampling_rate);
		f_printf(&p_sdcard_fileio->fdst, "%s\n",msg_buf);
		p_sdcard_fileio->is_fs_open = true;
		
		return true;
}

bool stop_record(void)
{
		uint8_t time_buf[6];
		if(!p_sdcard_fileio->is_fs_open)
		{//No file is recording
				return false;
		}
		
		memset(time_buf, 0, sizeof(time_buf));
		get_time(time_buf);

		//Close file
		f_sync(&p_sdcard_fileio->fdst);
		f_close(&p_sdcard_fileio->fdst);
		
		//Write summary info to the header
		open_file_w(p_sdcard_fileio->current_file);
		f_printf(&p_sdcard_fileio->fdst, "%06d data collected\n", data_num);
		f_printf(&p_sdcard_fileio->fdst, "Start:%02d:%02d:%02d.%02d\n",start_time[3],start_time[2],start_time[1],start_time[0]);
		f_printf(&p_sdcard_fileio->fdst, "End:%02d:%02d:%02d.%02d\n",time_buf[3],time_buf[2],time_buf[1],time_buf[0]);
		f_close(&p_sdcard_fileio->fdst);
		
		p_sdcard_fileio->is_fs_open = false;
		update_records_char();
		return true;
}

bool record(void)
{
			uint8_t meas_buf[20];
			uint8_t len = 0;
			int16_t v_buf;
	
			if(!p_sdcard_fileio->is_fs_open)
				return false;
							
			len = get_mpu_measurement(meas_buf)/2;
			
			f_printf(&p_sdcard_fileio->fdst, "%d:", data_num);


			//Format measurements to uint32
			for(int k = 0;k<len;k++ )
			{
					v_buf = ((meas_buf[2*k] & 0xff)<< 8) | ((meas_buf[2*k+1]) & 0xff); 
					f_printf(&p_sdcard_fileio->fdst," %d", v_buf);				
			}
			f_printf(&p_sdcard_fileio->fdst, "\n");			
			
				 
			if(data_num%1000 == 0)
					f_sync(&p_sdcard_fileio->fdst);  //Flush cache every 1000 data
				
			data_num++;
			nrf_delay_ms(p_mpu_config->sampling_rate);//Delay for sampling rate control
			
			return true;
}


