/*
 * mpu6050.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Blah
 */

#include <mpu6500.h>

static uint8_t MPU6500_Addr;

MPU6500_Status_t MPU6500_ReadByte(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, uint8_t reg_addr, uint8_t* pData) {
	if (HAL_I2C_Mem_Read(hi2c1, dev_addr << 1, reg_addr, 1, pData, 1, 500) == HAL_OK) {
		return MPU6500_OK;
	}
	return MPU6500_ERR;
}

MPU6500_Status_t MPU6500_WriteByte(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, uint8_t reg_addr, uint8_t* pData) {
	if (HAL_I2C_Mem_Write(hi2c1, dev_addr << 1, reg_addr, 1, pData, 1, 500) == HAL_OK) {
		return MPU6500_OK;
	}
	return MPU6500_ERR;
}

MPU6500_Status_t MPU6500_ReadBurst(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, uint8_t reg_base_addr, uint8_t* pDataBuffer, uint8_t len) {
	if (HAL_I2C_Mem_Read(hi2c1, dev_addr << 1, reg_base_addr, 1, pDataBuffer, len, 500) == HAL_OK) {
		return MPU6500_OK;
	}
	return MPU6500_ERR;
}

MPU6500_Status_t MPU6500_Read_Accelerometer_Data(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr, MPU6500_Accelerometer_Data_t* accel_data) {
	uint8_t data[6];
	if (MPU6500_ReadBurst(hi2c1, MPU6500_Addr, MPU6500_REG_ACCEL_BASE, data, sizeof(data)) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	accel_data->x = data[0] << 8 | data[1];
	accel_data->y = data[2] << 8 | data[3];
	accel_data->z = data[4] << 8 | data[5];
	return MPU6500_OK;
}

MPU6500_Accelerometer_Data_t MPU6500_Accelerometer_Calibration(const MPU6500_Accelerometer_Data_t *error_offset, MPU6500_Accelerometer_Data_t *data) {
	MPU6500_Accelerometer_Data_t calibration = {0};
	calibration.x = data->x - error_offset->x;
	calibration.y = data->y - error_offset->y;
	calibration.z = data->z - error_offset->z;
	return calibration;
}

MPU6500_Status_t MPU6500_LowPassFilter_Config(I2C_HandleTypeDef* hi2c1, uint8_t LFP_Val) {
	uint8_t val = 0;
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_FILTER_CONFIG, &val) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	val &= ~(0x7);
	val |= (uint8_t)LFP_Val;
	if (MPU6500_WriteByte(hi2c1, MPU6500_Addr, MPU6500_REG_FILTER_CONFIG, &val) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	return MPU6500_OK;
}



MPU6500_Status_t MPU6500_Interrupt_Config(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_Config_t level) {
	uint8_t int_cfg = 0;
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_INTPIN_CONFIG, &int_cfg) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	int_cfg &= ~0x80;
	int_cfg |= (uint8_t) level;
	return MPU6500_WriteByte(hi2c1, MPU6500_Addr, MPU6500_REG_INTPIN_CONFIG, &int_cfg);
}

MPU6500_Status_t MPU6500_Enable_Interrupts(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_t interrupts) {
	uint8_t cur_int_settings = 0;
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_EN, &cur_int_settings) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	cur_int_settings |= (uint8_t) interrupts;
	return MPU6500_WriteByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_EN, &cur_int_settings);
}

MPU6500_Status_t MPU6500_Disable_Interrupts(I2C_HandleTypeDef* hi2c1, MPU6500_Interrupt_t interrupts) {
	uint8_t cur_int_settings = 0;
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_EN, &cur_int_settings) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	cur_int_settings &= ~((uint8_t) interrupts);
	return MPU6500_WriteByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_EN, &cur_int_settings);
}

static MPU6500_Status_t MPU6500_Read_Interrupt_Status_Reg(I2C_HandleTypeDef* hi2c1, uint8_t* data) {
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_STATUS, data) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	return MPU6500_OK;
}

static MPU6500_Status_t MPU6500_Read_InterruptEnable_Status_Reg(I2C_HandleTypeDef* hi2c1, uint8_t* data) {
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_INT_EN, data) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	return MPU6500_OK;
}

MPU6500_Status_t MPU6500_Interrupt_Handle(I2C_HandleTypeDef* hi2c1) {
	uint8_t interrupt_status = 0, interrupt_settings = 0;
	if (MPU6500_Read_Interrupt_Status_Reg(hi2c1, &interrupt_status) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	if (MPU6500_Read_InterruptEnable_Status_Reg(hi2c1, &interrupt_settings) != MPU6500_OK) {
		return MPU6500_ERR;
	}

	if ((interrupt_settings & MOT_INT) && (interrupt_status & MOT_INT)) {
		MPU6500_MotionDetection_Callback();
	} else if ((interrupt_settings & RAW_RDY_INT) && (interrupt_status & RAW_RDY_INT)) {
		MPU6500_RawReady_Callback();
	} else if ((interrupt_settings & I2C_MST_INT) && (interrupt_status & I2C_MST_INT)) {
		MPU6500_I2C_MST_Callback();
	} else if ((interrupt_settings & FIFO_OFLOW_INT) && (interrupt_status & FIFO_OFLOW_INT)) {
		MPU6500_FIFO_OFLOW_Callback();
	}
	return MPU6500_OK;

}

MPU6500_Status_t MPU6500_Init(I2C_HandleTypeDef* hi2c1, uint8_t dev_addr) {
	MPU6500_Addr = dev_addr;
	uint8_t byte;
	if (MPU6500_ReadByte(hi2c1, MPU6500_Addr, MPU6500_REG_WHOAMI, &byte) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	if (byte != 0x70) {
		return MPU6500_ERR;
	}
	byte = 0x00;
	if (MPU6500_WriteByte(hi2c1, MPU6500_Addr, MPU6500_REG_PWR_MGMT1, &byte) != MPU6500_OK) {
		return MPU6500_ERR;
	}
	return MPU6500_OK;

}

void __weak MPU6500_MotionDetection_Callback() {

}

void __weak MPU6500_RawReady_Callback() {

}

void __weak MPU6500_I2C_MST_Callback() {

}

void __weak MPU6500_FIFO_OFLOW_Callback() {

}
