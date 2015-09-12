#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"

#define MPU9150_W_ADDRESS								0xD0
#define MPU9150_R_ADDRESS								0xD1	
#define AK8975C_W_ADDRESS								0x18
#define AK8975C_R_ADDRESS								0x19

/**MPU9150 and AK8975C registers addresses*/

#define MPU9150_REG_SMPLRT_DIV					0x19
#define MPU9150_REG_CONFIG							0x1A
#define MPU9150_REG_GYRO_CONFIG					0x1B
#define MPU9150_REG_ACCEL_CONFIG				0x1C
#define MPU9150_REG_FIFO_EN							0x23
#define MPU9150_REG_INT_PIN_CFG					0x37
#define MPU9150_REG_INT_ENABLE					0x38
#define MPU9150_REG_ACCEL_XOUT					0x3B
#define MPU9150_REG_TEMP_OUT						0x41
#define MPU9150_REG_GYRO_XOUT						0x43
#define	MPU9150_REG_USER_CTRL						0x6A
#define MPU9150_REG_PWR_MGMT_1					0x6B

#define	AK8975C_CTRL										0x0A
#define AK8975C_XOUT										0x03

/**Register configuration parameters*/
#define	DEFAULT_CONFIG									0x01
#define	DEFAULT_SMPLTR_DIV							0x18
#define	DEFAULT_FIFO_EN									0x00
#define	DEFAULT_INT_PIN_CONFIG					0x02
#define	DEFAULT_INT_ENABLE							0x00
#define	DEFAULT_USER_CTRL								0x00
#define DEFAULT_PWR_MGMT_1							0x00


#define SCALE_ACC_MIN										0x00
#define SCALE_ACC_NORMAL								0x08
#define SCALE_ACC_MAX										0x10

#define SCALE_GYRO_MIN									0x08
#define SCALE_GYRO_NORMAL								0x10
#define SCALE_GYRO_MAX									0x18


/**MPU configuration related commands*/
#define COMMAND_ENABLE_MEASUREMENT      0x11
#define COMMAND_SET_ACC_SCALE           0x12
#define COMMAND_SET_GYRO_SCALE          0x13
#define COMMAND_SET_SAMPLING_RATE       0x14
#define COMMAND_SET_ACC_MODE            0x15
#define COMMAND_SET_GYRO_MODE           0x16

//Command parameters
#define ENABLE_ACC											0x00
#define DISABLE_ACC											0x01
#define ENABLE_GYRO											0x02
#define DISABLE_GYRO										0x03
#define ENABLE_MAG											0x04
#define DISABLE_MAG											0x05
#define ENABLE_TEMP											0x06
#define DISABLE_TEMP										0x07

#define SAMPLING_RATE_MAX								1
#define SAMPLING_RATE_NORMAL						2
#define SAMPLING_RATE_MIN								4

typedef struct mpu_config_s mpu_config_t;

typedef struct mpu_config_s
{
		uint32_t							sampling_rate;
		bool									acc_enable;
		bool									gyro_enable;
		bool									mag_enable;
		bool									temp_enable;
		uint8_t								acc_scale;
		uint8_t								gyro_scale;
		
		uint8_t								acc_mode;
		uint8_t								gyro_mode;
	
}mpu_config_t;


void i2c_write(uint8_t reg, uint8_t value);

void i2c_read(uint8_t reg, uint8_t * buf, uint8_t len);

void mpu_config_command(uint8_t command, uint8_t param);

void mpu9150_init(mpu_config_t * p_config);

