/*
 * mpu6050.c
 *
 *  Created on: Oct 4, 2024
 *      Author: Yusuf
 */
#include "mpu6050.h"
#include <stdio.h>


MPU6050_Status_t MPU6050_Read_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *buf){
	return (HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR << 1,  reg_addr,1, buf, 1, MPU6050_I2C_TIMEOUT) == HAL_OK) ? MPU6050_OK : MPU6050_ERR;
}

MPU6050_Status_t MPU6050_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_base_addr, uint8_t *buf, uint32_t nbytes){
	return (HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR << 1,  reg_base_addr, 1, buf, nbytes, MPU6050_I2C_TIMEOUT) == HAL_OK) ? MPU6050_OK : MPU6050_ERR;
}

MPU6050_Status_t MPU6050_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data){
	return (HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, reg_addr, 1, data, 1, MPU6050_I2C_TIMEOUT) == HAL_OK) ? MPU6050_OK : MPU6050_ERR;

}

MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr){
	// read 1 byte from the sensor
	uint8_t buf =0;
	if (MPU6050_Read_Byte(hi2c, MPU6050_REG_WHOAMI, &buf) != MPU6050_OK){
		return  MPU6050_ERR;
	}

	if (buf == 0x68){
		printf("MPU6050 Read Byte Successful\n");
	}else{
		printf("MPU6050 Not Found\n");
		return MPU6050_ERR;
	}

	uint8_t data = 0x00;
	// MPU6050 is at sleep mode upon power-on, we have to wake it up.
	if (MPU6050_Write_Byte(hi2c, MPU6050_REG_PWRMGMT_1, &data) != MPU6050_OK){
			return  MPU6050_ERR;
	}

	return MPU6050_OK;
 }


MPU6050_Status_t MPU6050_Read_Accelerometer_Data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, MPU6050_Accel_Data_t *accelerometer_data){
	uint8_t raw_data[6];
	if (MPU6050_Read(hi2c, MPU6050_ACCELEROMETER_BASEADDR, raw_data, sizeof(raw_data)) != MPU6050_OK){
		return MPU6050_ERR;
	}

	accelerometer_data->x = (int16_t)(raw_data[0] << 8 | raw_data[1]);

	accelerometer_data->y = (int16_t)(raw_data[2] << 8 | raw_data[3]);

	accelerometer_data->z = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	return MPU6050_OK;
}

MPU6050_Accel_Data_t MPU6050_Accelerometer_Calibration(const MPU6050_Accel_Data_t *error_offset, MPU6050_Accel_Data_t *raw_data ){
	MPU6050_Accel_Data_t calibrated_data = {0};

	calibrated_data.x = raw_data->x - error_offset->x;
	calibrated_data.y = raw_data->y - error_offset->y;
	calibrated_data.z = raw_data->z - error_offset->z;

	return calibrated_data;
}

MPU6050_Status_t MPU6050_Configure_Low_Pass_Filter(I2C_HandleTypeDef *hi2c, MPU6050_DLPF_Config_t dlpf){
	// get current config data
	uint8_t config = 0;
	if (MPU6050_Read_Byte(hi2c, MPU6050_REG_CONFIG, &config) != MPU6050_OK){
		return MPU6050_ERR;
	}

	// reset the dlpf part of the config
	config &= ~(0x7);
	config |= (uint8_t)dlpf;

	if (MPU6050_Write_Byte(hi2c, MPU6050_REG_CONFIG, &config) != MPU6050_OK){
		return MPU6050_ERR;
	}

	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_Interrupt_Config(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_Config_t level){
	uint8_t int_cfg = 0;

	// read current configuration first
	if(MPU6050_Read_Byte(hi2c, MPU6050_REG_INT_PIN_CFG, &int_cfg) != MPU6050_OK){
		return MPU6050_ERR;
	}

	int_cfg &= ~0x80;		// Clear bit 7
	int_cfg |= (uint8_t)level;

	// Write the updated configuration back to the register
	return MPU6050_Write_Byte(hi2c, MPU6050_REG_INT_PIN_CFG, &int_cfg);
}

MPU6050_Status_t MPU6050_Enable_Interrupt(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_t interrupt){
	uint8_t current_int_settings = 0;
	if(MPU6050_Read_Byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings) != MPU6050_OK){
		return MPU6050_ERR;
	}

	current_int_settings |= (uint8_t)interrupt;

	return MPU6050_Write_Byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings);
}

MPU6050_Status_t MPU6050_Disable_Interrupt(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_t interrupt){
	uint8_t current_int_settings = 0;
	if(interrupt != (uint8_t)ALL_INT){
		MPU6050_Read_Byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings);
	}

	current_int_settings &= ~interrupt;

	return MPU6050_Write_Byte(hi2c, MPU6050_REG_INT_EN, &current_int_settings);
}

static MPU6050_Status_t get_interrupt_status(I2C_HandleTypeDef *hi2c, uint8_t *data){
	/// read the interrupt status register MPU6050_REG_INT_STATUS
	return MPU6050_Read_Byte(hi2c, MPU6050_REG_INT_STATUS, data);
}

static MPU6050_Status_t get_interrupt_settings(I2C_HandleTypeDef *hi2c, uint8_t *data){
	/// read the interrupt status register MPU6050_REG_INT_EN
	return MPU6050_Read_Byte(hi2c, MPU6050_REG_INT_EN, data);
}

void MPU6050_Interrupt_Handle(I2C_HandleTypeDef *hi2c){
	// read the interrupt status register of the sensor
	uint8_t interrupt_status;
	uint8_t interrupt_settings;

	get_interrupt_status(hi2c, &interrupt_status);
	get_interrupt_settings(hi2c, &interrupt_settings);

	// if raw data interrupt exists in the settings and it has occured
	if( (interrupt_settings & RAW_RDY_INT) && (interrupt_status & RAW_RDY_INT)){
		mpu6050_raw_data_ready_callback();
	}
}

__weak void mpu6050_raw_data_ready_callback(void){

}
