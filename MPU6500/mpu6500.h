/*
 * mpu6050.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Blah
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "main.h"

typedef enum {
	MPU6500_OK,
	MPU6500_ERR
} MPU6500_Status_t;

typedef enum {
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW = 0x01,
} MPU6500_Interrupt_Config_t;

typedef enum {
	RAW_RDY_INT = 0x01,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	MOT_INT = 0x40,
	ALL_INT = 0xFF
} MPU6500_Interrupt_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} MPU6500_Accelerometer_Data_t;

MPU6500_Status_t MPU6500_Init(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr);
MPU6500_Status_t MPU6500_ReadByte(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, uint8_t reg_addr, uint8_t* pData);
MPU6500_Status_t MPU6500_Read_Accelerometer_Data(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, MPU6500_Accelerometer_Data_t* accel_data);
MPU6500_Status_t MPU6500_ReadBurst(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, uint8_t reg_base_addr, uint8_t* pDataBuffer, uint8_t len);
MPU6500_Accelerometer_Data_t MPU6500_Accelerometer_Calibration(const MPU6500_Accelerometer_Data_t *error_offset, MPU6500_Accelerometer_Data_t *data);
MPU6500_Status_t MPU6500_Interrupt_Config(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_Config_t level);
MPU6500_Status_t MPU6500_Enable_Interrupts(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_t interrupts);
MPU6500_Status_t MPU6500_Disable_Interrupts(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_t interrupts);
MPU6500_Status_t MPU6500_Interrupt_Handle(I2C_HandleTypeDef* hi2c1);
MPU6500_Status_t MPU6500_LowPassFilter_Config(I2C_HandleTypeDef* hi2c1, uint8_t LFP_Val);

void MPU6500_MotionDetection_Callback();

void MPU6500_RawReady_Callback();

void MPU6500_I2C_MST_Callback();

void MPU6500_FIFO_OFLOW_Callback();

#define MPU6500_I2C_ADDR 0x68
#define MPU6500_REG_WHOAMI (uint8_t)117
#define MPU6500_REG_PWR_MGMT1 (uint8_t)107
#define MPU6500_REG_ACCEL_BASE (uint8_t)59
#define MPU6500_REG_FILTER_CONFIG (uint8_t)26
#define MPU6500_REG_INTPIN_CONFIG 0x37
#define MPU6500_REG_INT_STATUS 0x3A
#define MPU6500_REG_INT_EN 0x38
#define MPU6500_REG_MOT_THR 0x1F
#define MPU6500_REG_MOT_DUR 0x20



#endif /* INC_MPU6500_H_ */
