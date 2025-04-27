/*
 * mpu6050.h
 *
 *  Created on: Oct 4, 2024
 *      Author: Yusuf
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <stm32f4xx_hal.h>

#define MPU6050_I2C_ADDR			0x68			/*!<AD0 Pin is grounded>*/

#define MPU6050_I2C_TIMEOUT 		500UL


#define MPU6050_REG_WHOAMI			0x75		/*!<This register is used to verify the identity of the device. The contents of WHO_AM_I are the upper 6
												bits of the MPU-60X0’s 7-bit I2C address. The least significant bit of the MPU-60X0’s I2C address is
												determined by the value of the AD0 pin. The value of the AD0 pin is not reflected in this register.
												The default value of the register is 0x68. >*/

#define MPU6050_REG_PWRMGMT_1		0x6B		/*!<This register allows the user to configure the power mode and clock source. It also provides a bit for
												resetting the entire device, and a bit for disabling the temperature sensor>*/

#define MPU6050_REG_CONFIG			0x1A		/*!<This register configures the external Frame Synchronization (FSYNC)[5:3] pin sampling and the Digital
												Low Pass Filter (DLPF)[2:0] setting for both the gyroscopes and accelerometers.>*/
#define MPU6050_REG_INT_STATUS		0x3A
#define MPU6050_REG_INT_EN			0x38
#define MPU6050_REG_MOT_THR			0x1F
#define MPU6050_REG_MOT_DUR			0x20
#define MPU6050_REG_INT_PIN_CFG		0x37


#define MPU6050_ACCELEROMETER_BASEADDR	0x3B
#define MPU6050_ACCEL_X_1				0x3B 		/*!<ACCEL_XOUT[15:8]>*/
#define MPU6050_ACCEL_X_0				0x3C 		/*!<ACCEL_XOUT[7:0]>*/
#define MPU6050_ACCEL_Y_1				0x3D 		/*!<ACCEL_YOUT[15:8]>*/
#define MPU6050_ACCEL_Y_0				0x3E 		/*!<ACCEL_YOUT[7:0]>*/
#define MPU6050_ACCEL_Z_1				0x3F 		/*!<ACCEL_ZOUT[15:8]>*/
#define MPU6050_ACCEL_Z_0				0x40 		/*!<ACCEL_ZOUT[7:0]>*/

#define MPU6050_ACCEL_CALIBRATION_X		884			/*!< Calibration value for x axis, derived from analyzing the STM32Cube Monitor>*/
#define MPU6050_ACCEL_CALIBRATION_Y		-336		/*!< Calibration value for y axis, derived from analyzing the STM32Cube Monitor>*/
#define MPU6050_ACCEL_CALIBRATION_Z		-652		/*!< Calibration value for z axis, derived from analyzing the STM32Cube Monitor>*/



typedef enum{
	MPU6050_OK,
	MPU6050_ERR
}MPU6050_Status_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}MPU6050_Accel_Data_t;

typedef enum{
	MPU6050_DLPF_260Hz,										/*!<Delay: 0 ms		>*/
	MPU6050_DLPF_184Hz,										/*!<Delay: 2.0 ms	>*/
	MPU6050_DLPF_94Hz,										/*!<Delay: 3.0 ms	>*/
	MPU6050_DLPF_44Hz,										/*!<Delay: 4.9 ms	>*/
	MPU6050_DLPF_21Hz,										/*!<Delay: 8.5 ms	>*/
	MPU6050_DLPF_10Hz,										/*!<Delay: 13.8 ms	>*/
	MPU6050_DLPF_5Hz										/*!<Delay: 19.0 ms	>*/
}MPU6050_DLPF_Config_t;


typedef enum{
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW
}MPU6050_Interrupt_Config_t;

typedef enum{
	RAW_RDY_INT = 0x01,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	MOT_INT =0x40,
	ALL_INT = 0xFF
}MPU6050_Interrupt_t;


MPU6050_Status_t MPU6050_Read_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *buf);
MPU6050_Status_t MPU6050_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_base_addr, uint8_t *buf, uint32_t nbytes);
MPU6050_Status_t MPU6050_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data);
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr);
MPU6050_Status_t MPU6050_Read_Accelerometer_Data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, MPU6050_Accel_Data_t *accelerometer_data);
MPU6050_Status_t MPU6050_Configure_Low_Pass_Filter(I2C_HandleTypeDef *hi2c, MPU6050_DLPF_Config_t dlpf);
MPU6050_Accel_Data_t MPU6050_Accelerometer_Calibration(const MPU6050_Accel_Data_t *error_offset, MPU6050_Accel_Data_t *raw_data );
MPU6050_Status_t MPU6050_Interrupt_Config(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_Config_t level);
MPU6050_Status_t MPU6050_Enable_Interrupt(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_t interrupt);
MPU6050_Status_t MPU6050_Disable_Interrupt(I2C_HandleTypeDef *hi2c, MPU6050_Interrupt_t interrupt);
void MPU6050_Interrupt_Handle(I2C_HandleTypeDef *hi2c);


void mpu6050_raw_data_ready_callback(void);

#endif /* INC_MPU6050_H_ */
