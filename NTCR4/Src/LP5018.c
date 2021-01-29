/*
 * LP5018.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Luke
 */
#include "LP5018.h"
#include "main.h"
#include "stm32f0xx_hal.h"

uint8_t write_Reg(I2C_HandleTypeDef *hi2c1, uint8_t regAddr, uint8_t writeData, uint8_t length){

	static uint8_t devAddress = 0b0101000;
	uint8_t packet[2] = {0};
	packet[0] = regAddr;
	packet[1] = writeData;

	if(HAL_I2C_Master_Transmit(hi2c1, devAddress<<1, packet, length, 1) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t write_Continuous_Regs(I2C_HandleTypeDef *hi2c1, uint8_t regAddr, uint8_t *writeData, uint8_t length){

	static uint8_t devAddress = 0b0101000;
	uint8_t packet[10] = {0};
	for(uint8_t i = 0; i < length; i++ ){
		packet[i] = writeData[i+1];
	}

	if(HAL_I2C_Master_Transmit(hi2c1, devAddress<<1, packet, length, 1) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t read_Reg(I2C_HandleTypeDef *hi2c1, uint8_t *regAddr, uint8_t *rxdata, uint8_t length){

	static uint8_t devAddress = 0b0101000;

	if(HAL_I2C_Master_Transmit(hi2c1, devAddress<<1, regAddr, 1, 1) != HAL_OK)
	{
		return 0;
	}
	if(HAL_I2C_Master_Receive(hi2c1, devAddress<<1, rxdata, length, 1) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

void configLEDs(I2C_HandleTypeDef *hi2c1, uint8_t ledsOn){
	write_Reg(hi2c1, REG_DEVICE_CONFIG0, 0x40, 2);
	write_Reg(hi2c1, REG_LED_CONFIG0, ledsOn, 2);
}

void setLEDsColor(I2C_HandleTypeDef *hi2c1, uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){
	write_Reg(hi2c1, REG_BANK_BRIGHTNESS, brightness, 2);
	write_Reg(hi2c1, REG_BANK_A_COLOR, red, 2);		//R
	write_Reg(hi2c1, REG_BANK_B_COLOR, green, 2);	//G
	write_Reg(hi2c1, REG_BANK_C_COLOR, blue, 2);	//B
}
