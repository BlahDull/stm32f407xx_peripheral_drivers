/*
 * ds1307.c
 *
 *  Created on: Dec 5, 2025
 *      Author: Blah
 */

#include "ds1307.h"

I2C_Handle_t DS1307_I2CHandle;

static void ds1307_i2c_pin_config() {
	GPIO_Handle_t I2C_Pins;
	memset(&I2C_Pins, 0, sizeof(I2C_Pins));
	// SDA PB7
	I2C_Pins.pGPIOx = DS1307_I2C_GPIO_PORT;
	I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&I2C_Pins);
	// SCL PB6
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	GPIO_Init(&I2C_Pins);
}

static void ds1307_i2c_config() {
	DS1307_I2CHandle.pI2Cx = DS1307_I2C;
	DS1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	DS1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&DS1307_I2CHandle);
}

static void ds1307_write(uint8_t value, uint8_t addr) {
	uint8_t tx[2];
	tx[0] = addr;
	tx[1] = value;
	I2C_MasterSendData(&DS1307_I2CHandle, tx, 2, DS1307_I2C_ADDR, 0);
}

static uint8_t ds1307_read(uint8_t addr) {
	uint8_t data;
	I2C_MasterSendData(&DS1307_I2CHandle, &addr, 1, DS1307_I2C_ADDR, 0);
	I2C_MasterReceiveData(&DS1307_I2CHandle, &data, 1, DS1307_I2C_ADDR, 0);
	return data;
}

// Returns 1 if CH = 1
// Returns 0 if CH = 0
uint8_t ds1307_init() {
	// init I2C pins
	ds1307_i2c_pin_config();
	// init I2c peripheral
	ds1307_i2c_config();
	// enable i2c perihperal
	I2C_PeripheralControl(DS1307_I2C, ENABLE);
	// make clock halt = 0
	ds1307_write(0x00, DS1307_ADDR_SEC);
	// read back clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return (clock_state >> 7) & 0x1;
}

static uint8_t binary_to_bcd(uint8_t bin_num) {
	uint8_t tens, ones;
	if (bin_num >= 10) {
		tens = bin_num / 10;
		ones = bin_num % 10;
		return ((tens << 4) | ones);
	}
	return bin_num;
}

static uint8_t bcd_to_binary(uint8_t bcd_num) {
	uint8_t tens, ones;
	tens = (bcd_num >> 4) * 10;
	ones = bcd_num & 0xF;
	return tens + ones;
}

void ds1307_set_current_time(RTC_time_t* rtc_time) {
	uint8_t seconds, minutes, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);
	minutes = binary_to_bcd(rtc_time->minutes);
	ds1307_write(minutes, DS1307_ADDR_MIN);
	hrs = binary_to_bcd(rtc_time->hours);
	if (rtc_time->time_format == TIME_FORMAT_24_HR) {
		hrs &= ~(1 << 6);
	} else {
		hrs |= (1 << 6);
		hrs = (rtc_time->time_format == TIME_FORMAT_12HR_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}
	ds1307_write(hrs, DS1307_ADDR_HRS);
}

void ds1307_get_current_time(RTC_time_t* rtc_time) {
	uint8_t seconds, hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	rtc_time->seconds = bcd_to_binary(seconds);
	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));
	hrs = ds1307_read(DS1307_ADDR_HRS);
	if (hrs & (1 << 6)) {
		// 12 hr
			rtc_time->time_format = (hrs & (1 << 5)) ? TIME_FORMAT_12HR_PM :  TIME_FORMAT_12HR_AM;
			hrs &= ~(0x3 << 5);
	} else {
		// 24 hr
		rtc_time->time_format = TIME_FORMAT_24_HR;
	}
	rtc_time->hours = bcd_to_binary(hrs);
}

void ds1307_set_current_date(RTC_date_t* rtc_date) {
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}

void ds1307_get_current_date(RTC_date_t* rtc_date) {
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

