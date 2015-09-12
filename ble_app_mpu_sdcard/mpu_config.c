#include <stdint.h>
#include <string.h>
#include "mpu_config.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ble_nrf6310_pins.h"

static mpu_config_t * p_mpu_config;

void i2c_write(uint8_t reg, uint8_t value)
{
		uint8_t i2c_w_buf[2];
	
		i2c_w_buf[0] = reg;
		i2c_w_buf[1] = value;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_w_buf, 2 ,TWI_ISSUE_STOP);
	
}

void i2c_read(uint8_t reg, uint8_t * buf, uint8_t len)
{
		uint8_t i2c_w_buf[2];
	
		if( reg == AK8975C_XOUT)
		{ 	//Read magnetometer measurement
				i2c_w_buf[0] = AK8975C_CTRL;
				i2c_w_buf[1] = 0x01;
				twi_master_transfer(AK8975C_W_ADDRESS, i2c_w_buf, 2 ,TWI_ISSUE_STOP);
				nrf_delay_ms(10);
				
				i2c_w_buf[0] = reg;
				twi_master_transfer(AK8975C_W_ADDRESS, i2c_w_buf, 1 ,TWI_ISSUE_STOP);
				twi_master_transfer(AK8975C_R_ADDRESS, buf, 6 ,TWI_ISSUE_STOP);
				
		}
		else
		{
				i2c_w_buf[0] = reg;
				twi_master_transfer(MPU9150_W_ADDRESS, i2c_w_buf, 1 ,TWI_ISSUE_STOP);
				twi_master_transfer(MPU9150_R_ADDRESS, buf, len ,TWI_ISSUE_STOP);
		}
}

void mpu_config_command(uint8_t command, uint8_t param)
{
		switch(command)
		{
			case COMMAND_ENABLE_MEASUREMENT:
				switch(param)
				{
					case ENABLE_ACC:
						p_mpu_config->acc_enable = true;
						break;
					
					case DISABLE_ACC:
						p_mpu_config->acc_enable = false;
						break;
					
					case ENABLE_GYRO:
						p_mpu_config->gyro_enable = true;
						break;
					
					case DISABLE_GYRO:
						p_mpu_config->gyro_enable = false;
						break;
					
					case ENABLE_MAG:
						p_mpu_config->mag_enable = true;
						break;
					
					case DISABLE_MAG:
					p_mpu_config->mag_enable = false;

						break;
					
					case ENABLE_TEMP:
						p_mpu_config->temp_enable = true;
						break;
					
					case DISABLE_TEMP:
						p_mpu_config->temp_enable = false;
						break;
					
					default:
            break;
																									
				}
				break;

			case COMMAND_SET_ACC_SCALE:
				if( param == SCALE_ACC_MIN || param == SCALE_ACC_NORMAL || param == SCALE_ACC_MAX)
				{
					p_mpu_config->acc_scale = param;
					i2c_write(MPU9150_REG_ACCEL_CONFIG, param);
				}
				break;
			
			case COMMAND_SET_GYRO_SCALE:
				if( param == SCALE_GYRO_MIN || param == SCALE_GYRO_NORMAL || param == SCALE_GYRO_MAX)
				{
					p_mpu_config->gyro_scale = param;
					i2c_write(MPU9150_REG_GYRO_CONFIG, param);
				}
				break;
			
			case COMMAND_SET_SAMPLING_RATE:
				if( param == SAMPLING_RATE_MIN || param == SAMPLING_RATE_NORMAL || param == SAMPLING_RATE_MAX)
				{
					p_mpu_config->sampling_rate = param*50;
				}
				break;		
        
			default:
        break;				
		}
}

/**@Brief Function for getting MPU measurements according to MPU configuration*/
uint16_t get_mpu_measurement(uint8_t *buf)
{
		uint16_t len = 0;
	
		if(p_mpu_config->acc_enable)
		{
				i2c_read(MPU9150_REG_ACCEL_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(p_mpu_config->gyro_enable)
		{
				i2c_read(MPU9150_REG_GYRO_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(p_mpu_config->mag_enable)
		{
				i2c_read(AK8975C_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(p_mpu_config->temp_enable)
		{
				i2c_read(MPU9150_REG_TEMP_OUT, buf+len, 2);
				len+=2;
			
		}		
		
		return len;
}

void mpu9150_init(mpu_config_t * p_config, char* id)
{
		twi_master_init();
		
		p_mpu_config = p_config;
	
		p_config->sampling_rate = 100;
		p_config->acc_enable = true;
		p_config->gyro_enable = true;
		p_config->mag_enable = false;
		p_config->temp_enable = false;
		p_config->acc_scale = SCALE_ACC_MIN;
		p_config->gyro_scale = SCALE_GYRO_MIN;
		
		p_config->id = id;
	
		//Initialize mpu9150 registers
		i2c_write(MPU9150_REG_SMPLRT_DIV, DEFAULT_SMPLTR_DIV);
		i2c_write(MPU9150_REG_CONFIG, DEFAULT_CONFIG);
		i2c_write(MPU9150_REG_FIFO_EN, DEFAULT_FIFO_EN);
		i2c_write(MPU9150_REG_INT_PIN_CFG, DEFAULT_INT_PIN_CONFIG);
		i2c_write(MPU9150_REG_INT_ENABLE, DEFAULT_INT_ENABLE);
		i2c_write(MPU9150_REG_USER_CTRL, DEFAULT_USER_CTRL);
		i2c_write(MPU9150_REG_PWR_MGMT_1, DEFAULT_PWR_MGMT_1);
		i2c_write(MPU9150_REG_GYRO_CONFIG, p_config->gyro_scale);
		i2c_write(MPU9150_REG_ACCEL_CONFIG, p_config->acc_scale);
	
		nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);
}
